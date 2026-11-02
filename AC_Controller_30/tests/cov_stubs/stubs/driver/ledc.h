#pragma once
#include "esp_err.h"
#include "hal/ledc_types.h"
#include <stdint.h>

typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_t timer_num;
    ledc_timer_bit_t duty_resolution;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg;
} ledc_timer_config_t;

typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_intr_type_t intr_type;
    ledc_timer_t timer_sel;
    uint32_t duty;
    int hpoint;
} ledc_channel_config_t;

static inline esp_err_t ledc_timer_config(const ledc_timer_config_t* timer_conf){(void)timer_conf;return ESP_OK;}
static inline esp_err_t ledc_channel_config(const ledc_channel_config_t* ledc_conf){(void)ledc_conf;return ESP_OK;}
static inline esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty){(void)speed_mode;(void)channel;(void)duty;return ESP_OK;}
static inline esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel){(void)speed_mode;(void)channel;return ESP_OK;}
static inline uint32_t ledc_get_duty(ledc_mode_t speed_mode, ledc_channel_t channel){(void)speed_mode;(void)channel;return 0;}
static inline esp_err_t ledc_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz){(void)speed_mode;(void)timer_num;(void)freq_hz;return ESP_OK;}
static inline uint32_t ledc_get_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num){(void)speed_mode;(void)timer_num;return 1000;}
static inline esp_err_t ledc_fade_func_install(int intr_alloc_flags){(void)intr_alloc_flags;return ESP_OK;}
static inline esp_err_t ledc_set_fade_with_time(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, int max_fade_time_ms){(void)speed_mode;(void)channel;(void)target_duty;(void)max_fade_time_ms;return ESP_OK;}
static inline esp_err_t ledc_fade_start(ledc_mode_t speed_mode, ledc_channel_t channel, int fade_mode){(void)speed_mode;(void)channel;(void)fade_mode;return ESP_OK;}
