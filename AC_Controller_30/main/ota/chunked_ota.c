#include "chunked_ota.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psa/crypto.h"
#include "cJSON.h"
#include "SafeFunctions/Jio_EA_Safe_Functions.h"
#include "http.h"
#include "configuration/configuration.h"
#include "wifiConnect.h"   // wifi_is_connected
#include "main.h"          // mac_Wifi[], FW_VER*

/* Shared OTA state. Formerly defined in ota/http.c (the legacy
   "smart downloader", now removed); relocated here because chunked_ota
   is the live OTA path. Declared extern in http.h and read/written by
   configuration, controller, modbus_rtu, http_server, log_capture and
   main. */
ota_params_t httpserver_params;
int          otataskstarted   = 0;
int          g_download_progress = 0;

static const char *TAG = "CHUNKED_OTA";

/* Sentinel value of httpserver_params.is_ota that means "OTA completed
   on the previous boot; please POST a success report once Wi-Fi is up,
   then clear me to 0". Set by the OTA success path before reboot
   (persisted via update_config_tag) and cleared at the end of
   pending_report_task (also persisted). */
#define IS_OTA_PENDING_SUCCESS 100

/* Map enum values to the short human-readable names the server expects. */
static const char *ota_status_name(ota_t s)
{
    switch (s) {
    case OTA_SUCCESS:   return "success";
    case OTA_STARTED:   return "downloading";   // first tick before any packets
    case OTA_ONGOING:   return "downloading";
    case OTA_RETRYING:  return "retrying";
    case OTA_FAILED:    return "failure";
    case OTA_FLASHING:  return "flashing";
    case OTA_REBOOTING: return "rebooting";
    case OTA_IDLE:      return "idle";
    case OTA_DEFAULT:
    default:            return "default";
    }
}

static const char *ota_failure_name(ota_failure_t r)
{
    switch (r) {
    case OTA_INVALID_ARGUMENT:          return "invalid_argument";
    case OTA_BAD_JSON_INFO:             return "bad_json_info";
    case OTA_BAD_JSON_NEGOTIATE:        return "bad_json_negotiate";
    case OTA_MEM_ALLOC_FAILED:          return "mem_alloc_failed";
    case OTA_MEM_OTA_BEGIN_FAILED:      return "ota_begin_failed";
    case OTA_MEM_OTA_CLIENT_FAILED:     return "http_client_init_failed";
    case OTA_FAILED_T0_DOWNLOAD_PACKET: return "download_packet_failed";
    case OTA_FAILED_T0_WRITE_PACKET:    return "write_packet_failed";
    case OTA_SHA_MISMATCH:              return "sha_mismatch";
    case OTA_FAILED_T0_END:             return "ota_end_failed";
    case OTA_FAILED_T0_SET_BOOT:        return "set_boot_failed";
    case OTA_FAILED_TO_OPEN_CLIENT:     return "open_client_failed";
    case OTA_CONTENT_LENGTH_MISMATCH:   return "content_length_mismatch";
    case OTA_NO_CRC_PROVIDED:           return "no_crc_provided";
    case OTA_CRC_MISMATCH:              return "crc_mismatch";
    case OTA_HTTP_ERROR:                return "http_error";
    case OTA_FAILURE_DEFAULT:
    default:                            return "none";
    }
}

/* Helper: persist httpserver_params.is_ota=<new_state> back to the
   config file so the value survives reboot. Mirrors the JSON shape
   used elsewhere in this file. Non-static so log_capture and other
   sibling modules can clear the flag when their work finishes. */
void persist_is_ota(uint16_t new_state)
{
    httpserver_params.is_ota = new_state;
    /* worst case: 16-char wrapper + url(128) + 5 + filepath(64) + 5 == ~220 */
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"0\":{\"ota0\":\"%s:%d:%s:%u\"}}",
             httpserver_params.url,
             httpserver_params.port,
             httpserver_params.filepath,
             (unsigned) new_state);
    update_config_tag(config_path, "ota", buf);
    ESP_LOGI(TAG, "is_ota persisted = %u", (unsigned) new_state);
}

/* Build URL "<scheme>://<ip>:<port><path>" using httpserver_params.is_ota
   for the scheme, matching the firmware-download URL pattern. */
static int build_report_url(char *out, size_t cap,
                            const char *ip, uint16_t port, const char *path)
{
    const char *p = (path && path[0] != '\0') ? path : "/ota/report";
    const uint16_t pp = port ? port : (httpserver_params.is_ota == 2 ? 443 : 80);
    return snprintf(out, cap,
                    httpserver_params.is_ota == 2 ? "https://%s:%u%s%s"
                                                  : "http://%s:%u%s%s",
                    ip, (unsigned) pp,
                    p[0] == '/' ? "" : "/", p);
}

/* Single POST attempt — used by both the in-flight notifications (no prev,
   no post_reboot flag) and the post-reboot delivery (prev set, flag true). */
static esp_err_t post_report_impl(const char *ip, uint16_t port, const char *path,
                                  const char *filename,
                                  ota_t status, ota_failure_t reason,
                                  unsigned percentage,
                                  const char *prev_version,
                                  bool post_reboot)
{
    if (!ip || ip[0] == '\0') return ESP_OK;

    char url[320];
    int u = build_report_url(url, sizeof(url), ip, port, path);
    if (u <= 0 || u >= (int) sizeof(url)) return ESP_FAIL;

    char body[384];
    int n = snprintf(body, sizeof(body),
        "{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
         "\"status\":\"%s\",\"failure_reason\":\"%s\","
         "\"percentage\":%u,\"filename\":\"%s\","
         "\"firmware_version\":\"%d.%03d.%03d\"%s%s%s%s}",
        mac_Wifi[0], mac_Wifi[1], mac_Wifi[2],
        mac_Wifi[3], mac_Wifi[4], mac_Wifi[5],
        ota_status_name(status),
        ota_failure_name(reason),
        percentage,
        filename ? filename : "",
        FW_VERHIGH, FW_VERLOW, FW_VERPATCH,
        (prev_version && prev_version[0]) ? ",\"previous_firmware_version\":\"" : "",
        (prev_version && prev_version[0]) ? prev_version : "",
        (prev_version && prev_version[0]) ? "\"" : "",
        post_reboot ? ",\"post_reboot\":true" : "");
    if (n <= 0 || n >= (int) sizeof(body)) return ESP_FAIL;

    esp_http_client_config_t cfg = {
        .url = url, .method = HTTP_METHOD_POST, .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, n);
    esp_err_t err  = esp_http_client_perform(client);
    int       code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "report POST %s -> err=%d status=%d body=%s", url, err, code, body);
    return (err == ESP_OK && code >= 200 && code < 300) ? ESP_OK : ESP_FAIL;
}

/* Background task: wait for Wi-Fi, then POST the success report using
   ip/port/path from ota_report_* globals and filename from
   httpserver_params.filepath. Retries up to 30 attempts (~15 minutes).
   On success OR permanent give-up, is_ota is cleared back to 0 (in
   memory + persisted to config) so the marker doesn't re-trigger. */
static void pending_report_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "pending report: waiting for Wi-Fi (up to 5 min)...");
    int waits = 0;
    while (!wifi_is_connected && waits < 150) {       // 150 * 2s = 5 min
        vTaskDelay(pdMS_TO_TICKS(2000));
        waits++;
    }
    if (!wifi_is_connected) {
        ESP_LOGW(TAG, "pending report: Wi-Fi never came up; is_ota=100 kept for next boot");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "pending report: Wi-Fi up after %d s, settling 3s", waits * 2);
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (int attempt = 1; attempt <= 30; attempt++) {
        ESP_LOGI(TAG, "pending report: POST attempt %d/30", attempt);
        /* Report goes to the same host:port as the firmware server. */
        if (post_report_impl(httpserver_params.url,
                             httpserver_params.port,
                             ota_report_path,
                             httpserver_params.filepath,
                             OTA_SUCCESS, OTA_FAILURE_DEFAULT,
                             100, NULL, true) == ESP_OK) {
            ESP_LOGI(TAG, "pending report delivered after %d attempt(s)", attempt);
            persist_is_ota(0);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(30000));   // 30s between attempts
    }

    ESP_LOGE(TAG, "pending report: giving up after 30 attempts; clearing is_ota");
    persist_is_ota(0);
    vTaskDelete(NULL);
}

esp_err_t chunked_ota_send_pending_report(void)
{
    if (httpserver_params.is_ota != IS_OTA_PENDING_SUCCESS) return ESP_OK;
    ESP_LOGI(TAG, "is_ota=%d detected; spawning post-reboot success-report task",
             IS_OTA_PENDING_SUCCESS);
    BaseType_t ok = xTaskCreate(pending_report_task, "ota_pending_rpt",
                                6144, NULL, 4, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

esp_err_t chunked_ota_post_report(const char *report_ip,
                                  uint16_t   report_port,
                                  const char *report_path,
                                  const char *filename,
                                  ota_t status,
                                  ota_failure_t reason)
{
    return post_report_impl(report_ip, report_port, report_path, filename,
                            status, reason,
                            (unsigned) httpserver_params.ota_percentage,
                            NULL, false);
}

/* Plain CRC-16/Modbus (poly 0xA001, init 0xFFFF). Local copy so we don't
   inherit the byte-order handling in modbus_rtu.c's modbus_crc16(). */
static uint16_t ota_crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t) data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

typedef struct {
    char *json_buf;
    int   json_cap;
    int   json_len;
    char  crc16_hex[8];
    bool  got_crc;
} ota_http_ctx_t;

static esp_err_t http_event_cb(esp_http_client_event_t *evt)
{
    ota_http_ctx_t *ctx = (ota_http_ctx_t *) evt->user_data;
    if (!ctx) return ESP_OK;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_HEADER:
        if (evt->header_key && evt->header_value &&
            strcasecmp(evt->header_key, "X-CRC16") == 0) {
            size_t cap = sizeof(ctx->crc16_hex);
            size_t n   = strlen(evt->header_value);
            if (n >= cap) n = cap - 1;
            memcpy(ctx->crc16_hex, evt->header_value, n);
            ctx->crc16_hex[n] = '\0';
            ctx->got_crc = true;
        }
        break;
    case HTTP_EVENT_ON_DATA:
        if (ctx->json_buf && ctx->json_cap > 0) {
            int space = ctx->json_cap - ctx->json_len - 1;
            if (space > 0) {
                int n = evt->data_len < space ? evt->data_len : space;
                memcpy(ctx->json_buf + ctx->json_len, evt->data, n);
                ctx->json_len += n;
                ctx->json_buf[ctx->json_len] = '\0';
            }
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t fetch_json(const char *url, char *out, int out_cap, int *out_len)
{
    ota_http_ctx_t ctx = { .json_buf = out, .json_cap = out_cap, .json_len = 0 };
    esp_http_client_config_t cfg = {
        .url           = url,
        .timeout_ms    = 10000,
        .event_handler = http_event_cb,
        .user_data     = &ctx,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;
    esp_err_t err  = esp_http_client_perform(client);
    int       code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (err != ESP_OK || code != 200) {
        ESP_LOGE(TAG, "fetch %s err=%d status=%d", url, err, code);
        return ESP_FAIL;
    }
    if (out_len) *out_len = ctx.json_len;
    return ESP_OK;
}

static esp_err_t parse_info(const char *json, size_t *out_size, uint8_t out_sha[32],
                            uint16_t *out_min, uint16_t *out_max)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) return ESP_FAIL;
    cJSON *sz = cJSON_GetObjectItemCaseSensitive(root, "size");
    cJSON *sh = cJSON_GetObjectItemCaseSensitive(root, "sha256");
    cJSON *mn = cJSON_GetObjectItemCaseSensitive(root, "min_packet_size");
    cJSON *mx = cJSON_GetObjectItemCaseSensitive(root, "max_packet_size");
    if (!cJSON_IsNumber(sz) || !cJSON_IsString(sh) || strlen(sh->valuestring) != 64) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    *out_size = (size_t) sz->valuedouble;
    for (int i = 0; i < 32; i++)
        sscanf(sh->valuestring + 2 * i, "%02hhx", &out_sha[i]);
    if (out_min) *out_min = cJSON_IsNumber(mn) ? (uint16_t) mn->valuedouble : 256;
    if (out_max) *out_max = cJSON_IsNumber(mx) ? (uint16_t) mx->valuedouble : 4096;
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t parse_negotiate(const char *json, uint16_t *out_pkt, int *out_total)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) return ESP_FAIL;
    cJSON *pkt = cJSON_GetObjectItemCaseSensitive(root, "packet_size");
    cJSON *tot = cJSON_GetObjectItemCaseSensitive(root, "total_packets");
    if (!cJSON_IsNumber(pkt) || !cJSON_IsNumber(tot)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    *out_pkt   = (uint16_t) pkt->valuedouble;
    *out_total = (int) tot->valuedouble;
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t download_chunk(esp_http_client_handle_t client,
                                ota_http_ctx_t *ctx,
                                const char *url,
                                size_t start, size_t end,
                                uint8_t *buf, size_t *got_len)
{
    char range[48];
    snprintf(range, sizeof(range), "bytes=%u-%u", (unsigned) start, (unsigned) end);

    ctx->got_crc = false;
    ctx->crc16_hex[0] = '\0';

    esp_http_client_set_url(client, url);
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Range", range);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "open failed err=0x%x", err);
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_FAILED_TO_OPEN_CLIENT;
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status         = esp_http_client_get_status_code(client);
    size_t want        = end - start + 1;
    if (status != 206 || content_length < 0 || (size_t) content_length != want) {
        ESP_LOGE(TAG, "bad chunk reply status=%d cl=%d want=%u",
                 status, content_length, (unsigned) want);
        esp_http_client_close(client);
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_CONTENT_LENGTH_MISMATCH;
        return ESP_FAIL;
    }

    size_t read_total = 0;
    while (read_total < want) {
        int n = esp_http_client_read(client,
                                     (char *) (buf + read_total),
                                     want - read_total);
        if (n <= 0) {
            ESP_LOGE(TAG, "read failed n=%d total=%u/%u",
                     n, (unsigned) read_total, (unsigned) want);
            esp_http_client_close(client);
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_CONTENT_LENGTH_MISMATCH;
            return ESP_FAIL;
        }
        read_total += n;
    }
    esp_http_client_close(client);

    if (!ctx->got_crc) {
        ESP_LOGE(TAG, "server did not send X-CRC16");
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_NO_CRC_PROVIDED;
        return ESP_FAIL;
    }

    uint16_t server_crc = (uint16_t) strtoul(ctx->crc16_hex, NULL, 16);
    uint16_t local_crc  = ota_crc16_modbus(buf, want);
    if (server_crc != local_crc) {
        ESP_LOGW(TAG, "CRC mismatch server=%04x local=%04x", server_crc, local_crc);
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_CRC_MISMATCH;
        return ESP_ERR_INVALID_CRC;
    }

    if (got_len) *got_len = want;
    return ESP_OK;
}

esp_err_t chunked_ota_run(const chunked_ota_config_t *cfg)
{
    /* OTA source comes from httpserver_params (loaded from config file). */
    if (!cfg || httpserver_params.url[0] == '\0'
             || httpserver_params.filepath[0] == '\0')
    {
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_INVALID_ARGUMENT;
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t max_session = cfg->max_session_retries ? cfg->max_session_retries : 3;
    const uint8_t max_pkt     = cfg->max_packet_retries  ? cfg->max_packet_retries  : 3;

    /* Build "<scheme>://<host>:<port>" once; reused for /info, /negotiate, /firmware. */
    char base_url[260];
    snprintf(base_url, sizeof(base_url),
             httpserver_params.is_ota == 2 ? "https://%s:%u" : "http://%s:%u",
             httpserver_params.url, httpserver_params.port);
    const char *filename = httpserver_params.filepath;

    /* worst case GCC sees: base_url(260) + "/negotiate?file=" + filename(64)
       + "&packet_size=" + uint16 == ~360. Round up to 400 for headroom. */
    char url[400];
    char json[512];
    int  json_len = 0;

    snprintf(url, sizeof(url), "%s/info?file=%s", base_url, filename);
    if (fetch_json(url, json, sizeof(json), &json_len) != ESP_OK) return ESP_FAIL;

    size_t   total_size = 0;
    uint8_t  expected_sha[32];
    uint16_t srv_min = 256, srv_max = 4096;
    if (parse_info(json, &total_size, expected_sha, &srv_min, &srv_max) != ESP_OK) {
        ESP_LOGE(TAG, "bad info json");
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_BAD_JSON_INFO;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "info: size=%u min=%u max=%u", (unsigned) total_size, srv_min, srv_max);

    uint16_t want_pkt = cfg->requested_pkt_size ? cfg->requested_pkt_size : 2048;
    snprintf(url, sizeof(url), "%s/negotiate?file=%s&packet_size=%u",
             base_url, filename, want_pkt);
    if (fetch_json(url, json, sizeof(json), &json_len) != ESP_OK) return ESP_FAIL;

    uint16_t pkt_size     = 0;
    int      total_packets = 0;
    if (parse_negotiate(json, &pkt_size, &total_packets) != ESP_OK || pkt_size == 0) {
        ESP_LOGE(TAG, "bad negotiate json");
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_BAD_JSON_NEGOTIATE;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "negotiated: packet_size=%u total_packets=%d", pkt_size, total_packets);

    /* Notify: OTA started — sent once per chunked_ota_run() invocation. */
    httpserver_params.ota_percentage = 0;
    chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                            filename, OTA_STARTED, OTA_FAILURE_DEFAULT);

    /* worst case GCC sees: base_url(260) + "/firmware?file=" + filename(64) == ~340 */
    char fw_url[400];
    snprintf(fw_url, sizeof(fw_url), "%s/firmware?file=%s", base_url, filename);

    uint8_t *buf = malloc(pkt_size);
    if (!buf) {
        ESP_LOGE(TAG, "alloc %u failed", pkt_size);
        httpserver_params.ota_status = OTA_FAILED;
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = OTA_MEM_ALLOC_FAILED;
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = ESP_FAIL;
    int last_pct_5 = -1;   // last 5%-bucket we reported; reset per session
    for (uint8_t session = 0; session < max_session; session++) {
        ESP_LOGI(TAG, "===== session attempt %u/%u =====", session + 1, max_session);
        httpserver_params.ota_percentage = 0;
        httpserver_params.ota_failure_reason = 0;
        last_pct_5 = -1;

        /* Notify: restarting from packet 0 after a prior session failed. */
        if (session > 0) {
            chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                                    filename, OTA_RETRYING, OTA_FAILURE_DEFAULT);
        }

        const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
        if (!part) { ESP_LOGE(TAG, "no OTA partition"); break; }

        esp_ota_handle_t handle = 0;
        if (esp_ota_begin(part, total_size, &handle) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed");
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_MEM_OTA_BEGIN_FAILED;
            break;
        }

        psa_hash_operation_t sha = PSA_HASH_OPERATION_INIT;
        psa_hash_setup(&sha, PSA_ALG_SHA_256);

        ota_http_ctx_t ctx = { .json_buf = NULL };
        esp_http_client_config_t client_cfg = {
            .url                 = fw_url,
            .timeout_ms          = 30000,         // was 15000 — give slow links headroom
            .event_handler       = http_event_cb,
            .user_data           = &ctx,
            .keep_alive_enable   = true,
            .keep_alive_idle     = 5,
            .keep_alive_interval = 30,
            .keep_alive_count    = 3,
        };
        esp_http_client_handle_t client = esp_http_client_init(&client_cfg);
        if (!client) {
            esp_ota_abort(handle);
            psa_hash_abort(&sha);
            ESP_LOGE(TAG, "http client init failed");
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_MEM_OTA_CLIENT_FAILED;
            break;
        }

        bool session_ok = true;
        for (int idx = 0; idx < total_packets && session_ok; idx++) {
            size_t start = (size_t) idx * pkt_size;
            size_t end   = start + pkt_size - 1;
            if (end >= total_size) end = total_size - 1;
            size_t got = 0;

            esp_err_t pkt_err = ESP_FAIL;
            for (uint8_t attempt = 0; attempt < max_pkt; attempt++) {
                pkt_err = download_chunk(client, &ctx, fw_url, start, end, buf, &got);
                if (pkt_err == ESP_OK) break;
                ESP_LOGW(TAG, "pkt %d attempt %u/%u failed (0x%x)",
                         idx, attempt + 1, max_pkt, pkt_err);

                /* The previous attempt left the keep-alive cache pointing at
                   a dead TCP socket — retrying on the same handle just hits
                   the same timeout. Tear it down and rebuild before retrying. */
                esp_http_client_cleanup(client);
                vTaskDelay(pdMS_TO_TICKS(500 * (attempt + 1)));   // small backoff
                client = esp_http_client_init(&client_cfg);
                if (!client) {
                    ESP_LOGE(TAG, "http client re-init failed");
                    break;
                }
            }
            if (pkt_err != ESP_OK) {
                ESP_LOGE(TAG, "pkt %d: retries exhausted, abandoning session", idx);
                session_ok = false;
                httpserver_params.ota_status = OTA_FAILED;
                httpserver_params.ota_percentage = 0;
                httpserver_params.ota_failure_reason = OTA_FAILED_T0_DOWNLOAD_PACKET;
                break;
            }

            if (esp_ota_write(handle, buf, got) != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_write failed at idx=%d", idx);
                session_ok = false;
                httpserver_params.ota_status = OTA_FAILED;
                httpserver_params.ota_percentage = 0;
                httpserver_params.ota_failure_reason = OTA_FAILED_T0_WRITE_PACKET;
                break;
            }
            psa_hash_update(&sha, buf, got);
            float per = (float)(idx+1);
            per*= 100;
            per/= total_packets;
            httpserver_params.ota_percentage = (uint16_t) per;
            ESP_LOGI(TAG, "pkt %d/%d ok (%u bytes),percentage: %d%%",
                     idx + 1, total_packets, (unsigned) got, httpserver_params.ota_percentage);

            /* Notify: downloading — every time we cross a 10% boundary. */
            int cur_5 = httpserver_params.ota_percentage / 10;
            if (cur_5 != last_pct_5 && httpserver_params.ota_percentage < 100) {
                last_pct_5 = cur_5;
                chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                                        filename, OTA_ONGOING, OTA_FAILURE_DEFAULT);
            }
        }

        esp_http_client_cleanup(client);

        if (!session_ok) {
            esp_ota_abort(handle);
            psa_hash_abort(&sha);
            continue;
        }

        uint8_t calc_sha[32] = {0};
        size_t calc_len = 0;
        psa_hash_finish(&sha, calc_sha, sizeof(calc_sha), &calc_len);
        psa_hash_abort(&sha);

        if (memcmp(calc_sha, expected_sha, 32) != 0) {
            ESP_LOGE(TAG, "whole-file SHA-256 mismatch; restarting from packet 0");
            esp_ota_abort(handle);
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_SHA_MISMATCH;
            continue;
        }

        /* Notify: SHA verified, about to esp_ota_end + esp_ota_set_boot_partition. */
        httpserver_params.ota_percentage = 100;
        chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                                filename, OTA_FLASHING, OTA_FAILURE_DEFAULT);

        if (esp_ota_end(handle) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed");
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_FAILED_T0_END;
            esp_ota_abort(handle);
            continue;
        }
        if (esp_ota_set_boot_partition(part) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed");
            httpserver_params.ota_status = OTA_FAILED;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = OTA_FAILED_T0_SET_BOOT;
            esp_ota_abort(handle);
            continue;
        }
        ESP_LOGI(TAG, "OTA complete on partition %s, rebooting in 3s", part->label);
        result = ESP_OK;
        break;
    }

    if (result == ESP_OK) {
        g_download_progress = 100;
        httpserver_params.ota_status = g_download_progress;
        persist_is_ota(IS_OTA_PENDING_SUCCESS);
        free(buf);
        /* The ":100" written to config above doubles as the pending
           success-report marker -- the new firmware will detect it via
           httpserver_params.is_ota == 100 and POST the report. */
        /* Notify: about to reboot. Last message that comes from the OLD image. */
        chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                                filename, OTA_REBOOTING, OTA_FAILURE_DEFAULT);
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    else
    {
        ESP_LOGI(TAG, "retry Exhaused the,clearing the ota flag");
        httpserver_params.is_ota=0;
        //"{\"0\":{\"ota0\":\"0.0.0.0:8000:dummy.bin:0\"}}"
        persist_is_ota(0);
        free(buf);
         httpserver_params.ota_status = OTA_FAILED;
         httpserver_params.ota_percentage = 0;
        chunked_ota_post_report(cfg->report_ip, cfg->report_port, cfg->report_path,
                                filename, OTA_FAILED,
                                (ota_failure_t) httpserver_params.ota_failure_reason);
    }
    
    return result;
}

static void chunked_ota_task(void *arg)
{
    chunked_ota_config_t *cfg = (chunked_ota_config_t *) arg;
    esp_err_t err = chunked_ota_run(cfg);
    free(cfg);
    if (err != ESP_OK) {
        otataskstarted = 0;   // release gate so the periodic trigger can retry
    }
    vTaskDelete(NULL);
}

void chunked_ota_start_task(const chunked_ota_config_t *cfg)
{
    chunked_ota_config_t *copy = malloc(sizeof(*copy));
    if (!copy) {
        ESP_LOGE(TAG, "alloc cfg failed");
        return;
    }
    *copy = *cfg;
    otataskstarted = 1;   // latch gate; cleared by chunked_ota_task on failure
    xTaskCreate(chunked_ota_task, "chunked_ota", 8192, copy, 5, NULL);
}
