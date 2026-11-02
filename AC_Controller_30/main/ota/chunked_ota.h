#ifndef CHUNKED_OTA_H
#define CHUNKED_OTA_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* OTA source (url/port/scheme/filename) is read from the global
   httpserver_params at run-time. Only retry tuning and the report
   destination need to be passed in. */
typedef struct {
    /* Sized to hold httpserver_params.url[128] without truncation. */
    char     report_ip[128];        // POST destination host/IP, empty = skip report
    uint16_t report_port;           // POST destination port (0 -> 80 for http)
    char     report_path[64];       // POST URL path, empty -> "/ota/report"
    uint16_t requested_pkt_size;    // requested packet size (0 -> server default)
    uint8_t  max_session_retries;   // whole-file retries on SHA mismatch (0 -> 3)
    uint8_t  max_packet_retries;    // per-packet CRC retries (0 -> 3)
} chunked_ota_config_t;

typedef enum {
    OTA_DEFAULT = 0,
    OTA_SUCCESS,
    OTA_STARTED,
    OTA_ONGOING,
    OTA_RETRYING,
    OTA_FAILED,
    OTA_FLASHING,      // SHA verified, about to commit + set boot partition
    OTA_REBOOTING,     // commit done, esp_restart() coming next
    OTA_IDLE,          // heartbeat: device alive, about to start a periodic check
} ota_t;
typedef enum {
    OTA_FAILURE_DEFAULT = 0,
    OTA_INVALID_ARGUMENT,
    OTA_BAD_JSON_INFO,
    OTA_BAD_JSON_NEGOTIATE,
    OTA_MEM_ALLOC_FAILED,
    OTA_MEM_OTA_BEGIN_FAILED,
    OTA_MEM_OTA_CLIENT_FAILED,
    OTA_FAILED_T0_DOWNLOAD_PACKET,
    OTA_FAILED_T0_WRITE_PACKET,
    OTA_SHA_MISMATCH,
    OTA_FAILED_T0_END,
    OTA_FAILED_T0_SET_BOOT,
    OTA_FAILED_TO_OPEN_CLIENT,
    OTA_CONTENT_LENGTH_MISMATCH,
    OTA_NO_CRC_PROVIDED,
    OTA_CRC_MISMATCH,
    OTA_HTTP_ERROR,
} ota_failure_t;
esp_err_t chunked_ota_run(const chunked_ota_config_t *cfg);
void chunked_ota_start_task(const chunked_ota_config_t *cfg);

/* Set httpserver_params.is_ota=<new_state> in memory AND persist the
   value to the config file (so it survives reboot). Exposed so adjacent
   modules (e.g. log_capture) can clear the flag when they finish. */
void persist_is_ota(uint16_t new_state);

/* Fire-and-forget POST to report OTA completion or failure. Builds
   "http://<ip>:<port><path>" internally; pass path "" to default to
   "/ota/report". Returns ESP_OK on HTTP 2xx, ESP_FAIL otherwise. Safe
   to call with an empty ip (no-op). */
esp_err_t chunked_ota_post_report(const char *report_ip,
                                  uint16_t   report_port,
                                  const char *report_path,
                                  const char *filename,
                                  ota_t status,
                                  ota_failure_t reason);

/* If httpserver_params.is_ota == 100 (the post-reboot pending marker
   left by the previous firmware in the config file), spawn a task that
   waits for Wi-Fi and POSTs the success report with the *current*
   running firmware version, then clears is_ota back to 0. Safe to
   call repeatedly -- no-op unless is_ota == 100. */
esp_err_t chunked_ota_send_pending_report(void);

#ifdef __cplusplus
}
#endif
#endif
