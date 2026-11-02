#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct esp_timer* esp_timer_handle_t;
typedef void (*esp_timer_cb_t)(void* arg);

typedef struct {
    esp_timer_cb_t callback;
    void* arg;
    int dispatch_method;
    const char* name;
    bool skip_unhandled_events;
} esp_timer_create_args_t;

static inline esp_err_t esp_timer_create(const esp_timer_create_args_t* create_args, esp_timer_handle_t* out_handle){(void)create_args;if(out_handle)*out_handle=NULL;return ESP_OK;}
static inline esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us){(void)timer;(void)timeout_us;return ESP_OK;}
static inline esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period){(void)timer;(void)period;return ESP_OK;}
static inline esp_err_t esp_timer_stop(esp_timer_handle_t timer){(void)timer;return ESP_OK;}
static inline esp_err_t esp_timer_delete(esp_timer_handle_t timer){(void)timer;return ESP_OK;}
static inline int64_t esp_timer_get_time(void){return 0;}
static inline bool esp_timer_is_active(esp_timer_handle_t timer){(void)timer;return false;}
#define ESP_TIMER_TASK 0
