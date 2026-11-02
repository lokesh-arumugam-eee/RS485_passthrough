#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <time.h>

typedef enum { ESP_SNTP_SYNC_STATUS_RESET=0, ESP_SNTP_SYNC_STATUS_COMPLETED, ESP_SNTP_SYNC_STATUS_IN_PROGRESS } esp_sntp_sync_status_t;
typedef void (*esp_sntp_time_sync_notification_cb_t)(struct timeval *tv);

static inline void esp_sntp_init(void){}
static inline void esp_sntp_stop(void){}
static inline void esp_sntp_setservername(uint8_t idx, const char *server){(void)idx;(void)server;}
static inline void esp_sntp_setoperatingmode(uint8_t operating_mode){(void)operating_mode;}
static inline esp_sntp_sync_status_t esp_sntp_get_sync_status(void){return ESP_SNTP_SYNC_STATUS_COMPLETED;}
static inline void esp_sntp_set_time_sync_notification_cb(esp_sntp_time_sync_notification_cb_t callback){(void)callback;}
#define SNTP_OPMODE_POLL 0
#define SNTP_SYNC_MODE_IMMED 0
static inline void sntp_set_time_sync_notification_cb(esp_sntp_time_sync_notification_cb_t callback){(void)callback;}
