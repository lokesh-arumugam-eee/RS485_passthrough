#pragma once
#include "esp_err.h"
#include <stdint.h>

typedef enum { ESP_SLEEP_WAKEUP_UNDEFINED=0, ESP_SLEEP_WAKEUP_EXT0, ESP_SLEEP_WAKEUP_EXT1, ESP_SLEEP_WAKEUP_TIMER, ESP_SLEEP_WAKEUP_TOUCHPAD, ESP_SLEEP_WAKEUP_ULP } esp_sleep_source_t;
typedef esp_sleep_source_t esp_sleep_wakeup_cause_t;

static inline esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us){(void)time_in_us;return ESP_OK;}
static inline void esp_deep_sleep_start(void){}
static inline void esp_light_sleep_start(void){}
static inline esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(void){return ESP_SLEEP_WAKEUP_UNDEFINED;}
