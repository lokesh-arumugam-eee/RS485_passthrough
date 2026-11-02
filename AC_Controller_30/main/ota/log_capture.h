#ifndef LOG_CAPTURE_H
#define LOG_CAPTURE_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_CAPTURE_DEFAULT_DURATION_SEC  (5 * 60)        // 5 minutes
#define LOG_CAPTURE_MAX_BYTES             (64 * 1024)     // 64 KB hard cap
#define LOG_CAPTURE_MARK_INTERVAL_MS      30000           // time-marker every 30 s

/* Lifecycle values for log_params_t.status. Polled externally
   (Modbus / HTTP / debug print) to watch progress. */
#define LOG_STATUS_IDLE       0
#define LOG_STATUS_CAPTURING  1
#define LOG_STATUS_UPLOADING  2
#define LOG_STATUS_DONE       3
#define LOG_STATUS_FAILED     4

/* Decoupled from httpserver_params so log capture has its own
   lifecycle / config. IP+port come from httpserver_params (same
   firmware/log server). path/duration/trigger/status are owned here. */
typedef struct {
    char     path[64];          // URL path prefix, e.g. "fota/log"
    uint16_t duration_sec;      // capture window in seconds; 0 -> 300
    uint8_t  trigger;           // write 1 externally to start; auto-cleared by task
    uint8_t  status;            // see LOG_STATUS_* above
} log_params_t;

extern log_params_t log_params;

/* Gate flag used by main.c to ensure only one log_capture_task is alive
   at a time. Set to 1 before xTaskCreate, cleared back to 0 by the task
   itself when it finishes (success or failure). Separate from the OTA
   gate so log capture and OTA can be reasoned about independently. */
extern int ota_log_started;

/* Spawn a task that tees ESP_LOG* output to /spiffs/debug.log for
   log_params.duration_sec seconds, then PUTs the file to
   http(s)://<httpserver_params.url>:<port>/<log_params.path>/<mac>.log
   and clears log_params.trigger / sets status to DONE or FAILED. */
esp_err_t log_capture_start(void);

/* Call once during boot (after SPIFFS is mounted and the config file
   has populated log_params + httpserver_params). If a partial log
   file is sitting on flash from a crashed previous session, read it
   into RAM, delete the on-flash copy, and spawn a task that waits for
   Wi-Fi then PUTs the bytes to
   http(s)://<host>:<port>/<path>/<mac>_recovered.log
   so at least *some* of the captured logs reach the server. Returns
   ESP_OK whether or not a file was found. */
esp_err_t log_capture_recover_on_boot(void);

#ifdef __cplusplus
}
#endif
#endif
