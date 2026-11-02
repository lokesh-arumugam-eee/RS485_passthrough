#pragma once
#include "esp_err.h"
#include <stdint.h>

typedef enum { ESP_BLUEDROID_STATUS_UNINITIALIZED=0, ESP_BLUEDROID_STATUS_INITIALIZED, ESP_BLUEDROID_STATUS_ENABLED } esp_bluedroid_status_t;

static inline esp_err_t esp_bluedroid_init(void){return ESP_OK;}
static inline esp_err_t esp_bluedroid_deinit(void){return ESP_OK;}
static inline esp_err_t esp_bluedroid_enable(void){return ESP_OK;}
static inline esp_err_t esp_bluedroid_disable(void){return ESP_OK;}
static inline esp_bluedroid_status_t esp_bluedroid_get_status(void){return ESP_BLUEDROID_STATUS_ENABLED;}
