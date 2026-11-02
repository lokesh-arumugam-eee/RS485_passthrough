#ifndef http_H
#define http_H

#include <stdint.h>

/* Shared OTA configuration/status. Populated from the config file and
   over Modbus/HTTP; consumed by the chunked OTA path and log capture.
   The instance is defined in ota/chunked_ota.c. */
typedef struct {
    char url[128];
    uint16_t port;
    char filepath[64];
    uint16_t is_ota;
    uint16_t ota_status;
    uint16_t ota_percentage;
    uint16_t ota_failure_reason;
} ota_params_t;

extern ota_params_t httpserver_params;
extern int otataskstarted;
extern int g_download_progress;

#endif
