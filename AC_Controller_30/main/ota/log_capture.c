#include "log_capture.h"
#include "chunked_ota.h"   // (kept for symmetry; not strictly needed now)

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "http.h"          // httpserver_params (url, port, is_ota scheme)
#include "main.h"          // mac_Wifi[]
#include "configuration/configuration.h"   // file_exists, delete_config_file

extern bool wifi_is_connected;

/* Dedicated gate for log capture. main.c writes 1 before kicking the
   task, the task writes 0 in every exit path. Kept separate from
   otataskstarted (OTA gate) so log and OTA can coexist conceptually. */
int ota_log_started = 0;

static const char *TAG       = "LOG_CAPTURE";
static const char *LOG_PATH  = "/spiffs/debug.log";

/* Fill `out` with a "_YYYYMMDD_HHMMSS" suffix from current wall-clock
   time. If SNTP hasn't synced yet (year < 2024), fall back to
   "_uptime<seconds>s" so successive uploads still get unique names.
   The result always starts with '_' and contains only filename-safe
   characters (alphanumerics + underscore). */
static void make_time_suffix(char *out, size_t cap)
{
    time_t    now;
    struct tm tm_info;
    time(&now);
    localtime_r(&now, &tm_info);

    if (tm_info.tm_year + 1900 >= 2024) {
        strftime(out, cap, "_%Y%m%d_%H%M%S", &tm_info);
    } else {
        unsigned long up_s =
            (unsigned long) (xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
        snprintf(out, cap, "_uptime%lus", up_s);
    }
}

/* Build "<scheme>://<host>:<port>/<log_params.path>/<MAC>_<TS><suffix>.log".
   `suffix` may be "" or e.g. "_recovered". A timestamp is always
   injected between the MAC and the user suffix so successive uploads
   from the same device land in distinct files server-side. */
static int build_upload_url(char *out, size_t cap, const char *suffix)
{
    char mac[16];
    snprintf(mac, sizeof(mac), "%02X%02X%02X%02X%02X%02X",
             mac_Wifi[0], mac_Wifi[1], mac_Wifi[2],
             mac_Wifi[3], mac_Wifi[4], mac_Wifi[5]);

    char ts[32];
    make_time_suffix(ts, sizeof(ts));

    const char *p = log_params.path;
    while (*p == '/') p++;

    return snprintf(out, cap,
                    httpserver_params.is_ota == 2 ? "https://%s:%u/%s/%s%s%s.log"
                                                  : "http://%s:%u/%s/%s%s%s.log",
                    httpserver_params.url, httpserver_params.port,
                    p, mac, ts, suffix ? suffix : "");
}

/* Body-PUT helper: returns ESP_OK on HTTP 2xx, ESP_FAIL otherwise. */
static esp_err_t http_put_body(const char *url,
                               const void *body, size_t size,
                               const char *xhdr_key, const char *xhdr_val)
{
    esp_http_client_config_t cfg = {
        .url        = url,
        .method     = HTTP_METHOD_PUT,
        .timeout_ms = 30000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;
    esp_http_client_set_header(client, "Content-Type", "text/plain");
    if (xhdr_key && xhdr_val) {
        esp_http_client_set_header(client, xhdr_key, xhdr_val);
    }
    esp_http_client_set_post_field(client, (const char *) body, (int) size);
    esp_err_t err  = esp_http_client_perform(client);
    int       code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "PUT %s -> err=%d status=%d bytes=%u",
             url, err, code, (unsigned) size);
    return (err == ESP_OK && code >= 200 && code < 300) ? ESP_OK : ESP_FAIL;
}

/* Public, mutable. External code (Modbus / HTTP command / config loader)
   writes path/duration/trigger; this module owns status. */
log_params_t log_params = {
    .path         = "fota/log",
    .duration_sec = LOG_CAPTURE_DEFAULT_DURATION_SEC,
    .trigger      = 0,
    .status       = LOG_STATUS_IDLE,
};

static FILE             *s_log_file        = NULL;
static size_t            s_log_bytes       = 0;
static vprintf_like_t    s_original_vprintf = NULL;
static SemaphoreHandle_t s_log_mutex       = NULL;
static volatile bool     s_capturing       = false;

/* vprintf hook: tee to original sink (UART) AND copy into the file. */
static int log_capture_vprintf(const char *fmt, va_list args)
{
    int ret = 0;
    if (s_original_vprintf) {
        va_list cp;
        va_copy(cp, args);
        ret = s_original_vprintf(fmt, cp);
        va_end(cp);
    }

    if (!s_capturing || !s_log_file || s_log_bytes >= LOG_CAPTURE_MAX_BYTES) {
        return ret;
    }

    char buf[512];
    int  n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n <= 0) return ret;
    if (n >= (int) sizeof(buf)) n = sizeof(buf) - 1;

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_log_file && s_log_bytes < LOG_CAPTURE_MAX_BYTES) {
            size_t space    = LOG_CAPTURE_MAX_BYTES - s_log_bytes;
            size_t to_write = ((size_t) n > space) ? space : (size_t) n;
            fwrite(buf, 1, to_write, s_log_file);
            s_log_bytes += to_write;
            fflush(s_log_file);
        }
        xSemaphoreGive(s_log_mutex);
    }
    return ret;
}

/* Write a wall-clock timestamp directly to the file (NOT through the
   vprintf hook -- we don't want this on UART, only in the captured
   file). Falls back to a "(time not synced)" marker if SNTP hasn't
   set the clock yet. */
static void write_time_marker(void)
{
    if (!s_log_file || s_log_bytes >= LOG_CAPTURE_MAX_BYTES) return;

    time_t    now;
    struct tm tm_info;
    time(&now);
    localtime_r(&now, &tm_info);

    char ts[80];
    int  n;
    if (tm_info.tm_year + 1900 < 2024) {
        n = snprintf(ts, sizeof(ts),
                     "\n=== MARK uptime=%lus (clock not synced) ===\n",
                     (unsigned long) (xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
    } else {
        n = (int) strftime(ts, sizeof(ts),
                           "\n=== MARK %Y-%m-%d %H:%M:%S ===\n", &tm_info);
    }
    if (n <= 0) return;

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_log_file && s_log_bytes + n <= LOG_CAPTURE_MAX_BYTES) {
            fwrite(ts, 1, n, s_log_file);
            s_log_bytes += n;
            fflush(s_log_file);
        }
        xSemaphoreGive(s_log_mutex);
    }
}

static esp_err_t upload_log_file(void)
{
    FILE *f = fopen(LOG_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "could not open %s for read", LOG_PATH);
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        ESP_LOGW(TAG, "log file is empty, skipping upload");
        return ESP_OK;
    }

    char *body = malloc(size);
    if (!body) {
        fclose(f);
        ESP_LOGE(TAG, "alloc %ld failed", size);
        return ESP_ERR_NO_MEM;
    }
    size_t rd = fread(body, 1, size, f);
    fclose(f);
    if (rd != (size_t) size) {
        free(body);
        return ESP_FAIL;
    }

    char url[320];
    build_upload_url(url, sizeof(url), "");
    esp_err_t err = http_put_body(url, body, (size_t) size, NULL, NULL);
    free(body);
    return err;
}

/* Context handed to the recovery task -- the buffer is malloc'd by
   log_capture_recover_on_boot() and freed by the task before exit. */
typedef struct {
    uint8_t *buf;
    size_t   size;
} recover_ctx_t;

static void recover_task(void *arg)
{
    recover_ctx_t *ctx = (recover_ctx_t *) arg;

    ESP_LOGI(TAG, "recover: waiting for Wi-Fi (up to 5 min) to upload %u bytes",
             (unsigned) ctx->size);
    int waits = 0;
    while (!wifi_is_connected && waits < 150) {       // 5 min
        vTaskDelay(pdMS_TO_TICKS(2000));
        waits++;
    }
    if (!wifi_is_connected) {
        ESP_LOGW(TAG, "recover: Wi-Fi never came up; dropping %u bytes",
                 (unsigned) ctx->size);
        free(ctx->buf);
        free(ctx);
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));   // settle

    char url[320];
    build_upload_url(url, sizeof(url), "_recovered");

    /* 3 attempts, 10 s between. The data is already lossy (capture was
       interrupted) so don't grind for 15 min like the OTA-success path
       does -- best-effort delivery is fine. */
    esp_err_t up = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "recover: PUT attempt %d/3", attempt);
        up = http_put_body(url, ctx->buf, ctx->size,
                           "X-Log-Recovered", "1");
        if (up == ESP_OK) break;
        if (attempt < 3) vTaskDelay(pdMS_TO_TICKS(10000));
    }
    if (up == ESP_OK) {
        ESP_LOGI(TAG, "recover: delivered partial log (%u bytes)",
                 (unsigned) ctx->size);
    } else {
        ESP_LOGE(TAG, "recover: all 3 attempts failed; %u bytes lost",
                 (unsigned) ctx->size);
    }

    free(ctx->buf);
    free(ctx);
    vTaskDelete(NULL);
}

esp_err_t log_capture_recover_on_boot(void)
{
    /* Don't race a new capture that may already be in flight. */
    if (ota_log_started != 0) return ESP_OK;
    if (!file_exists(LOG_PATH)) return ESP_OK;

    FILE *f = fopen(LOG_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "leftover log found but could not open for read");
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        delete_config_file(LOG_PATH);
        return ESP_OK;
    }

    uint8_t *buf = malloc(size);
    if (!buf) {
        fclose(f);
        ESP_LOGE(TAG, "alloc %ld for recovery failed", size);
        return ESP_ERR_NO_MEM;
    }
    size_t rd = fread(buf, 1, size, f);
    fclose(f);
    if (rd != (size_t) size) {
        free(buf);
        return ESP_FAIL;
    }

    /* Got the bytes in RAM -- safe to delete from flash now so the next
       capture session starts with a clean file even if Wi-Fi takes
       forever to come up. */
    delete_config_file(LOG_PATH);
    ESP_LOGI(TAG, "found leftover log (%ld bytes), will upload as _recovered",
             size);

    recover_ctx_t *ctx = malloc(sizeof(*ctx));
    if (!ctx) { free(buf); return ESP_ERR_NO_MEM; }
    ctx->buf  = buf;
    ctx->size = (size_t) size;

    BaseType_t ok = xTaskCreate(recover_task, "log_recover",
                                6144, ctx, 4, NULL);
    if (ok != pdPASS) {
        free(buf);
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void log_capture_task(void *arg)
{
    (void) arg;
    uint32_t duration_sec = log_params.duration_sec
                          ? log_params.duration_sec
                          : LOG_CAPTURE_DEFAULT_DURATION_SEC;
    uint32_t duration_ms  = duration_sec * 1000U;

    if (!s_log_mutex) s_log_mutex = xSemaphoreCreateMutex();
    if (!s_log_mutex) {
        ESP_LOGE(TAG, "could not create mutex");
        log_params.status  = LOG_STATUS_FAILED;
        log_params.trigger = 0;
        ota_log_started    = 0;
        vTaskDelete(NULL);
        return;
    }

    /* Always start fresh -- delete any stale leftover. */
    if (file_exists(LOG_PATH)) {
        delete_config_file(LOG_PATH);
        ESP_LOGI(TAG, "deleted existing %s", LOG_PATH);
    }

    s_log_file = fopen(LOG_PATH, "w");
    if (!s_log_file) {
        ESP_LOGE(TAG, "could not open %s for write", LOG_PATH);
        log_params.status  = LOG_STATUS_FAILED;
        log_params.trigger = 0;
        ota_log_started    = 0;
        vTaskDelete(NULL);
        return;
    }
    s_log_bytes = 0;

    log_params.status = LOG_STATUS_CAPTURING;

    s_original_vprintf = esp_log_set_vprintf(log_capture_vprintf);
    s_capturing        = true;
    ESP_LOGI(TAG, "log capture started: duration=%lus, cap=%d bytes, mark every %d ms",
             (unsigned long) duration_sec, LOG_CAPTURE_MAX_BYTES,
             LOG_CAPTURE_MARK_INTERVAL_MS);

    /* Initial marker at t=0 plus one every LOG_CAPTURE_MARK_INTERVAL_MS. */
    write_time_marker();
    uint32_t elapsed = 0;
    while (elapsed < duration_ms && s_capturing) {
        uint32_t chunk = duration_ms - elapsed;
        if (chunk > LOG_CAPTURE_MARK_INTERVAL_MS) chunk = LOG_CAPTURE_MARK_INTERVAL_MS;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        elapsed += chunk;
        if (s_capturing) write_time_marker();
    }

    /* Stop capturing -- restore UART sink BEFORE closing the file. */
    s_capturing = false;
    if (s_original_vprintf) {
        esp_log_set_vprintf(s_original_vprintf);
        s_original_vprintf = NULL;
    }

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (s_log_file) {
            fclose(s_log_file);
            s_log_file = NULL;
        }
        xSemaphoreGive(s_log_mutex);
    }

    ESP_LOGI(TAG, "log capture stopped, %u bytes captured; uploading...",
             (unsigned) s_log_bytes);

    log_params.status = LOG_STATUS_UPLOADING;
    esp_err_t up_err  = upload_log_file();
    log_params.status = (up_err == ESP_OK) ? LOG_STATUS_DONE : LOG_STATUS_FAILED;

    /* Always remove the local file -- next capture starts clean. */
    if (file_exists(LOG_PATH)) {
        delete_config_file(LOG_PATH);
        ESP_LOGI(TAG, "removed %s", LOG_PATH);
    }

    /* Consume the trigger so we don't re-arm on the next main-loop tick,
       and release our log-capture gate so a fresh capture can be started. */
    log_params.trigger = 0;
    ota_log_started    = 0;
    vTaskDelete(NULL);
}

esp_err_t log_capture_start(void)
{
    BaseType_t ok = xTaskCreate(log_capture_task, "log_capture",
                                6144, NULL, 5, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}
