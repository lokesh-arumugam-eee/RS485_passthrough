#pragma once
#include "esp_err.h"
#include <stdint.h>
typedef void* esp_ping_handle_t;
typedef struct { uint32_t target_addr; uint32_t count; uint32_t interval_ms; uint32_t timeout_ms; uint32_t data_size; int tos; int ttl; void (*on_ping_success)(esp_ping_handle_t hdl, void *args); void (*on_ping_timeout)(esp_ping_handle_t hdl, void *args); void (*on_ping_end)(esp_ping_handle_t hdl, void *args); void *cb_args; } esp_ping_config_t;
#define ESP_PING_DEFAULT_CONFIG() {0,3,1000,1000,64,0,64,NULL,NULL,NULL,NULL}
static inline esp_err_t esp_ping_new_session(const esp_ping_config_t *config, const void *callbacks, esp_ping_handle_t *hdl_out){(void)config;(void)callbacks;if(hdl_out)*hdl_out=NULL;return ESP_OK;}
static inline esp_err_t esp_ping_start(esp_ping_handle_t hdl){(void)hdl;return ESP_OK;}
static inline esp_err_t esp_ping_stop(esp_ping_handle_t hdl){(void)hdl;return ESP_OK;}
static inline esp_err_t esp_ping_delete_session(esp_ping_handle_t hdl){(void)hdl;return ESP_OK;}
static inline esp_err_t esp_ping_get_profile(esp_ping_handle_t hdl, int profile, void *data, uint32_t size){(void)hdl;(void)profile;(void)data;(void)size;return ESP_OK;}
#define ESP_PING_PROF_SEQNO 0
#define ESP_PING_PROF_TTL 1
#define ESP_PING_PROF_IPADDR 2
#define ESP_PING_PROF_SIZE 3
#define ESP_PING_PROF_TIMEGAP 4
#define ESP_PING_PROF_DURATION 5
#define ESP_PING_PROF_REQUEST 6
#define ESP_PING_PROF_REPLY 7
