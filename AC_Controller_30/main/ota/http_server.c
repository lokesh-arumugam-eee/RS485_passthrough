#include "http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "cJSON.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include "main.h"
#include "configuration/configuration.h"
#include "ota/http.h"
#include "ota/log_capture.h"   // log_params + LOG_STATUS_* + log_capture_start
#include "controller.h"   // rebootcnter
#include "xmodem.h"       // delete_file_example()

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "http_cfg";

#define HTTP_CONFIRM_MAGIC      0x0A0A     // mirrors Modbus delete/reboot magic
#define HTTP_REQ_BODY_MAX       512        // enough for a full-config POST
#define CHUNK_BUF_SIZE          256        // stack buffer for streamed GET

/* ---------------------------------------------------------- */
/* Minimal-allocation response helpers                        */
/* ---------------------------------------------------------- */

/* Positive ack: send static literal, zero heap, zero stack copy. */
static esp_err_t send_ack(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_type(req, "application/json");
    if (!msg) {
        static const char ok[] = "{\"status\":\"ok\"}";
        return httpd_resp_send(req, ok, sizeof(ok) - 1);
    }
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "{\"status\":\"ok\",\"message\":\"%s\"}", msg);
    if (n < 0 || n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    return httpd_resp_send(req, buf, n);
}

static esp_err_t send_nack(httpd_req_t *req, const char *field, const char *reason)
{
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    char buf[160];
    int n;
    if (field && reason) {
        n = snprintf(buf, sizeof(buf),
                     "{\"status\":\"error\",\"field\":\"%s\",\"reason\":\"%s\"}",
                     field, reason);
    } else if (reason) {
        n = snprintf(buf, sizeof(buf),
                     "{\"status\":\"error\",\"reason\":\"%s\"}", reason);
    } else {
        static const char err[] = "{\"status\":\"error\"}";
        return httpd_resp_send(req, err, sizeof(err) - 1);
    }
    if (n < 0 || n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    return httpd_resp_send(req, buf, n);
}

/* Read full request body into a heap buffer (caller frees). */
static char *read_body(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > HTTP_REQ_BODY_MAX) return NULL;

    char *buf = malloc(total + 1);
    if (!buf) return NULL;
    int off = 0;
    while (off < total) {
        int n = httpd_req_recv(req, buf + off, total - off);
        if (n <= 0) {
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
            free(buf);
            return NULL;
        }
        off += n;
    }
    buf[off] = '\0';
    return buf;
}

/* ---------------------------------------------------------- */
/* Validation helpers                                         */
/* ---------------------------------------------------------- */

static bool get_int_in_range(const cJSON *root, const char *key,
                             int lo, int hi, int *out, bool *present)
{
    cJSON *it = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!it)              { *present = false; return true; }
    if (!cJSON_IsNumber(it)) { *present = true; return false; }
    int v = it->valueint;
    if (v < lo || v > hi) { *present = true; return false; }
    *out = v;
    *present = true;
    return true;
}

static bool get_string(const cJSON *root, const char *key,
                       const char **out, bool *present)
{
    cJSON *it = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!it) { *present = false; return true; }
    if (!cJSON_IsString(it) || !it->valuestring) { *present = true; return false; }
    *out = it->valuestring;
    *present = true;
    return true;
}

static bool parse_ipv4(const char *s, uint32_t *out)
{
    return inet_pton(AF_INET, s, out) == 1;
}

/* ---------------------------------------------------------- */
/* Persistence (mirrors modbus_rtu.c writeback format)        */
/* ---------------------------------------------------------- */

typedef struct {
    bool urt;
    bool ip;
    bool ota;
} write_dirty_t;

static void persist_urt(void)
{
    char s[128];
    sprintf_j(s, sizeof(s),
              "{\"0\":{\"curt0\":\"%d:%lu:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\"}}",
              system_urts[0].urt_id,        (unsigned long)system_urts[0].baud,
              system_urts[0].parity,        system_urts[0].data_bits,
              system_urts[0].stop_bits,     system_urts[0].retry_cnt,
              system_urts[0].retry_period,  system_urts[0].DisconnectonNoA,
              system_urts[0].DefConnStatus, system_urts[0].InterwriteDelay,
              system_urts[0].InterReadDelay,system_urts[0].AutoConnect);
    update_config_tag(config_path, "urt", s);
}

static void persist_ip(void)
{
    char s[200];
    /* Named fields — same keys the "ip" section of this endpoint already
       accepts, and what configuration.c's ipc parser now expects (BUG-FIX
       #18: replaced the old positional "ipc0" colon string, which silently
       mis-parsed on a missing/reordered field). */
    sprintf_j(s, sizeof(s),
              "{\"0\":{\"static_ip\":\"" IPSTR "\",\"gw\":\"" IPSTR "\",\"nm\":\"" IPSTR "\","
              "\"mb_port\":%d,\"pt_port\":%d,\"http_port\":%d,\"timeout\":%d,\"pt_retry\":%d,\"static_en\":%d}}",
              IP2STR(&ip_config[0].st_ip_info.ip),
              IP2STR(&ip_config[0].st_ip_info.gw),
              IP2STR(&ip_config[0].st_ip_info.netmask),
              ip_config[0].port,
              ip_config[0].pass_through_port,
              ip_config[0].http_port,
              ip_config[0].server_timeout,
              ip_config[0].Passthr_retry_cnt,
              ip_config[0].St_ip_En_dis);
    update_config_tag(config_path, "ipc", s);
}

static void persist_ota(void)
{
    char s[256];
    uint16_t trigger = httpserver_params.is_ota;
    if(trigger==100)
    {
        trigger =1;
    }
    sprintf_j(s, sizeof(s),
              "{\"0\":{\"ota0\":\"%s:%d:%s:%d\"}}",
              httpserver_params.url, httpserver_params.port,
              httpserver_params.filepath, trigger);
    update_config_tag(config_path, "ota", s);
}

/* ---------------------------------------------------------- */
/* GET /api/config — streamed, zero-heap                      */
/* ---------------------------------------------------------- */
static esp_err_t cfg_get_handler(httpd_req_t *req)
{
    char  buf[CHUNK_BUF_SIZE];
    int   n;

    httpd_resp_set_type(req, "application/json");

    /* info (RO) */
    n = snprintf(buf, sizeof(buf),
        "{\"info\":{\"fw_ver\":\"%d.%03d.%03d\",\"rel_dt\":\"%02d/%02d/%04d\","
        "\"wifi_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ble_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"},",
        FW_VERHIGH, FW_VERLOW, FW_VERPATCH, REL_DA, REL_MO, REL_YR,
        mac_Wifi[0], mac_Wifi[1], mac_Wifi[2], mac_Wifi[3], mac_Wifi[4], mac_Wifi[5],
        mac_ble[0],  mac_ble[1],  mac_ble[2],  mac_ble[3],  mac_ble[4],  mac_ble[5]);
    httpd_resp_send_chunk(req, buf, n);

    /* uart (R/W) */
    n = snprintf(buf, sizeof(buf),
        "\"uart\":{\"baud\":%lu,\"parity\":%d,\"dbits\":%d,\"sbits\":%d,"
        "\"rcnt\":%d,\"rprd\":%d},",
        (unsigned long)system_urts[0].baud, system_urts[0].parity,
        system_urts[0].data_bits, system_urts[0].stop_bits,
        system_urts[0].retry_cnt, system_urts[0].retry_period);
    httpd_resp_send_chunk(req, buf, n);

    /* ip (R/W) — three IPs fit in one chunk */
    n = snprintf(buf, sizeof(buf),
        "\"ip\":{\"static_ip\":\"" IPSTR "\",\"gw\":\"" IPSTR "\","
        "\"nm\":\"" IPSTR "\",\"mb_port\":%d,\"pt_port\":%d,\"http_port\":%d,"
        "\"static_en\":%d,\"timeout\":%d,\"pt_retry\":%d},",
        IP2STR(&ip_config[0].st_ip_info.ip),
        IP2STR(&ip_config[0].st_ip_info.gw),
        IP2STR(&ip_config[0].st_ip_info.netmask),
        ip_config[0].port, ip_config[0].pass_through_port,
        ip_config[0].http_port, ip_config[0].St_ip_En_dis,
        ip_config[0].server_timeout, ip_config[0].Passthr_retry_cnt);
    httpd_resp_send_chunk(req, buf, n);

    /* ota (R/W) — url is up to 128B + file up to 64B, so split */
    httpd_resp_send_chunk(req, "\"ota\":{\"url\":\"", 14);
    httpd_resp_send_chunk(req, httpserver_params.url,
                          strlen(httpserver_params.url));
    n = snprintf(buf, sizeof(buf), "\",\"port\":%d,\"file\":\"",
                 httpserver_params.port);
    httpd_resp_send_chunk(req, buf, n);
    httpd_resp_send_chunk(req, httpserver_params.filepath,
                          strlen(httpserver_params.filepath));
    n = snprintf(buf, sizeof(buf), "\",\"trigger\":%d},",
                 httpserver_params.is_ota);
    httpd_resp_send_chunk(req, buf, n);

    /* log (R/W: trigger/duration/path; status RO) */
    httpd_resp_send_chunk(req, "\"log\":{\"path\":\"", 15);
    httpd_resp_send_chunk(req, log_params.path, strlen(log_params.path));
    n = snprintf(buf, sizeof(buf),
                 "\",\"duration_sec\":%u,\"trigger\":%u,\"status\":%u},",
                 log_params.duration_sec, log_params.trigger, log_params.status);
    httpd_resp_send_chunk(req, buf, n);

    /* fota_status (RO) */
    n = snprintf(buf, sizeof(buf),
        "\"fota_status\":{\"status\":%d,\"progress\":%d,\"fail\":%d}}",
        httpserver_params.ota_status, httpserver_params.ota_percentage,
        httpserver_params.ota_failure_reason);
    httpd_resp_send_chunk(req, buf, n);

    /* terminate chunked response */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ---------------------------------------------------------- */
/* POST /api/config                                           */
/* ---------------------------------------------------------- */
static esp_err_t cfg_post_handler(httpd_req_t *req)
{
    char *body = read_body(req);
    if (!body) return send_nack(req, NULL, "missing or oversize body");

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_nack(req, NULL, "invalid json");

    int   updated = 0;
    write_dirty_t dirty = {0};
    int   v = 0;
    const char *s = NULL;
    bool  present = false;

#define BAIL(field, reason) do { cJSON_Delete(root); return send_nack(req, field, reason); } while (0)
#define WRITE_INT(obj, key, lo, hi, sink, dirty_flag) do {                                  \
        if (!get_int_in_range(obj, key, lo, hi, &v, &present)) BAIL(key, "out of range");   \
        if (present) { sink = v; dirty_flag = true; updated++; }                            \
    } while (0)

    if (cJSON_GetObjectItemCaseSensitive(root, "info") ||
        cJSON_GetObjectItemCaseSensitive(root, "fota_status")) {
        BAIL(NULL, "read-only section");
    }

    /* --- uart --- */
    cJSON *uart = cJSON_GetObjectItemCaseSensitive(root, "uart");
    if (cJSON_IsObject(uart)) {
        if (!get_int_in_range(uart, "baud", 4800, 115200, &v, &present)) BAIL("baud", "out of range");
        if (present) { system_urts[0].baud = (uint32_t)v; dirty.urt = true; updated++; }
        WRITE_INT(uart, "parity",   0, 2,     system_urts[0].parity,       dirty.urt);
        WRITE_INT(uart, "dbits",    7, 8,     system_urts[0].data_bits,    dirty.urt);
        WRITE_INT(uart, "sbits",    1, 2,     system_urts[0].stop_bits,    dirty.urt);
        WRITE_INT(uart, "rcnt",     0, 255,   system_urts[0].retry_cnt,    dirty.urt);
        WRITE_INT(uart, "rprd",     0, 65535, system_urts[0].retry_period, dirty.urt);
    }

    /* --- ip --- */
    cJSON *ip = cJSON_GetObjectItemCaseSensitive(root, "ip");
    if (cJSON_IsObject(ip)) {
        if (!get_string(ip, "static_ip", &s, &present)) BAIL("static_ip", "must be string");
        if (present) {
            uint32_t a;
            if (!parse_ipv4(s, &a)) BAIL("static_ip", "invalid IPv4");
            ip_config[0].st_ip_info.ip.addr = a;
            dirty.ip = true; updated++;
        }
        if (!get_string(ip, "gw", &s, &present)) BAIL("gw", "must be string");
        if (present) {
            uint32_t a;
            if (!parse_ipv4(s, &a)) BAIL("gw", "invalid IPv4");
            ip_config[0].st_ip_info.gw.addr = a;
            dirty.ip = true; updated++;
        }
        if (!get_string(ip, "nm", &s, &present)) BAIL("nm", "must be string");
        if (present) {
            uint32_t a;
            if (!parse_ipv4(s, &a)) BAIL("nm", "invalid IPv4");
            ip_config[0].st_ip_info.netmask.addr = a;
            dirty.ip = true; updated++;
        }
        WRITE_INT(ip, "mb_port",   1, 65535, ip_config[0].port,              dirty.ip);
        WRITE_INT(ip, "pt_port",   1, 65535, ip_config[0].pass_through_port, dirty.ip);
        WRITE_INT(ip, "http_port", 1, 65535, ip_config[0].http_port,         dirty.ip);
        WRITE_INT(ip, "static_en", 0, 1,     ip_config[0].St_ip_En_dis,      dirty.ip);
        WRITE_INT(ip, "timeout",   1, 65535, ip_config[0].server_timeout,    dirty.ip);
        WRITE_INT(ip, "pt_retry",  0, 255,   ip_config[0].Passthr_retry_cnt, dirty.ip);
    }

    /* --- ota --- */
    cJSON *ota = cJSON_GetObjectItemCaseSensitive(root, "ota");
    if (cJSON_IsObject(ota)) {
        if (!get_string(ota, "url", &s, &present)) BAIL("url", "must be string");
        if (present) {
            if (strlen(s) >= sizeof(httpserver_params.url)) BAIL("url", "too long");
            memset_j(httpserver_params.url, sizeof(httpserver_params.url), 0,
                     sizeof(httpserver_params.url));
            strcpy_j(httpserver_params.url, sizeof(httpserver_params.url), s, strlen(s));
            dirty.ota = true; updated++;
        }
        WRITE_INT(ota, "port", 1, 65535, httpserver_params.port, dirty.ota);
        if (!get_string(ota, "file", &s, &present)) BAIL("file", "must be string");
        if (present) {
            if (strlen(s) >= sizeof(httpserver_params.filepath)) BAIL("file", "too long");
            memset_j(httpserver_params.filepath, sizeof(httpserver_params.filepath), 0,
                     sizeof(httpserver_params.filepath));
            strcpy_j(httpserver_params.filepath, sizeof(httpserver_params.filepath), s, strlen(s));
            dirty.ota = true; updated++;
        }
        WRITE_INT(ota, "trigger", 0, 2, httpserver_params.is_ota, dirty.ota);
    }

    /* --- log capture (RAM-only — not persisted to config file) --- */
    cJSON *log = cJSON_GetObjectItemCaseSensitive(root, "log");
    if (cJSON_IsObject(log)) {
        if (!get_string(log, "path", &s, &present)) BAIL("path", "must be string");
        if (present) {
            if (strlen(s) >= sizeof(log_params.path)) BAIL("path", "too long");
            memset_j(log_params.path, sizeof(log_params.path), 0, sizeof(log_params.path));
            strcpy_j(log_params.path, sizeof(log_params.path), s, strlen(s));
            updated++;
        }
        if (!get_int_in_range(log, "duration_sec", 1, 65535, &v, &present))
            BAIL("duration_sec", "out of range (1..65535)");
        if (present) { log_params.duration_sec = (uint16_t) v; updated++; }
        if (!get_int_in_range(log, "trigger", 0, 1, &v, &present))
            BAIL("trigger", "must be 0 or 1");
        if (present) { log_params.trigger = (uint8_t) v; updated++; }
    }

    if (updated == 0) BAIL(NULL, "no writable fields provided");

#undef BAIL
#undef WRITE_INT

    cJSON_Delete(root);

    if (dirty.urt) persist_urt();
    if (dirty.ip)  persist_ip();
    if (dirty.ota) persist_ota();

    /* Inline ack with count — no cJSON */
    char ackbuf[48];
    int an = snprintf(ackbuf, sizeof(ackbuf),
                      "{\"status\":\"ok\",\"updated\":%d}", updated);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, ackbuf, an);
}

/* ---------------------------------------------------------- */
/* POST /api/system/delete_config                             */
/* ---------------------------------------------------------- */
static esp_err_t cfg_delete_handler(httpd_req_t *req)
{
    char *body = read_body(req);
    if (!body) return send_nack(req, NULL, "missing body");

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_nack(req, NULL, "invalid json");

    cJSON *cj = cJSON_GetObjectItemCaseSensitive(root, "confirm");
    bool good = cJSON_IsNumber(cj) && cj->valueint == HTTP_CONFIRM_MAGIC;
    cJSON_Delete(root);

    if (!good) return send_nack(req, "confirm", "expected 2570 (0x0A0A)");

    ESP_LOGW(TAG, "delete_config requested via HTTP");
    if (delete_file_example() != 0) {
        return send_nack(req, NULL, "file delete failed");
    }
    return send_ack(req, "config deleted, reboot recommended");
}

/* ---------------------------------------------------------- */
/* POST /api/system/reboot                                    */
/* ---------------------------------------------------------- */
static esp_err_t cfg_reboot_handler(httpd_req_t *req)
{
    char *body = read_body(req);
    if (!body) return send_nack(req, NULL, "missing body");

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) return send_nack(req, NULL, "invalid json");

    cJSON *cj = cJSON_GetObjectItemCaseSensitive(root, "confirm");
    bool good = cJSON_IsNumber(cj) && cj->valueint == HTTP_CONFIRM_MAGIC;
    cJSON_Delete(root);

    if (!good) return send_nack(req, "confirm", "expected 2570 (0x0A0A)");

    ESP_LOGW(TAG, "reboot requested via HTTP");
    if (rebootcnter == 0) rebootcnter = 10;
    return send_ack(req, "reboot scheduled");
}

/* ---------------------------------------------------------- */
/* Init                                                       */
/* ---------------------------------------------------------- */
esp_err_t http_config_server_init(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = (ip_config && ip_config[0].http_port > 0)
                              ? ip_config[0].http_port : 80;
    cfg.max_uri_handlers = 8;
    cfg.max_open_sockets = 2;     // one admin client + one transient/keep-alive
    cfg.backlog_conn     = 2;     // listen() backlog
    cfg.lru_purge_enable = true;  // newest connection wins if 3rd arrives

    httpd_handle_t srv = NULL;
    esp_err_t rc = httpd_start(&srv, &cfg);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() failed on port %d: %d", cfg.server_port, rc);
        return rc;
    }

    httpd_uri_t uri_get_config = {
        .uri = "/api/config", .method = HTTP_GET,
        .handler = cfg_get_handler, .user_ctx = NULL,
    };
    httpd_uri_t uri_post_config = {
        .uri = "/api/config", .method = HTTP_POST,
        .handler = cfg_post_handler, .user_ctx = NULL,
    };
    httpd_uri_t uri_delete = {
        .uri = "/api/system/delete_config", .method = HTTP_POST,
        .handler = cfg_delete_handler, .user_ctx = NULL,
    };
    httpd_uri_t uri_reboot = {
        .uri = "/api/system/reboot", .method = HTTP_POST,
        .handler = cfg_reboot_handler, .user_ctx = NULL,
    };

    httpd_register_uri_handler(srv, &uri_get_config);
    httpd_register_uri_handler(srv, &uri_post_config);
    httpd_register_uri_handler(srv, &uri_delete);
    httpd_register_uri_handler(srv, &uri_reboot);

    ESP_LOGI(TAG, "HTTP config server listening on port %d", cfg.server_port);
    return ESP_OK;
}
