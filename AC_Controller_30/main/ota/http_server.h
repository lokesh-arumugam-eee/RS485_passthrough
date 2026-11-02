#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_err.h"

// Start the HTTP configuration server (listen port from ip_config[0].http_port).
// Endpoints:
//   GET  /api/config            -> read-all parameters (JSON)
//   POST /api/config            -> write one-or-more parameters (JSON body)
//   POST /api/system/delete_config -> deletes config file (requires confirm)
//   POST /api/system/reboot        -> reboots device      (requires confirm)
esp_err_t http_config_server_init(void);

#endif
