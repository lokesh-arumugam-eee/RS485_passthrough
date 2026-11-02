#pragma once
#include <stdint.h>
#include "esp_err.h"
static inline void esp_restart(void){}
static inline uint32_t esp_get_free_heap_size(void){return 100000;}
static inline uint32_t esp_get_minimum_free_heap_size(void){return 50000;}
typedef enum { ESP_CHIP_ESP32S3=9 } esp_chip_model_t;
typedef enum { ESP_RST_UNKNOWN=0, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW, ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO } esp_reset_reason_t;
static inline esp_reset_reason_t esp_reset_reason(void){return ESP_RST_POWERON;}
