#pragma once
#include "esp_err.h"
#include <stdint.h>
#include "soc/gpio_num.h"

typedef gpio_num_t gpio_num_t;
typedef enum { GPIO_MODE_DISABLE=0, GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2, GPIO_MODE_OUTPUT_OD=6, GPIO_MODE_INPUT_OUTPUT_OD=7, GPIO_MODE_INPUT_OUTPUT=3 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE=0, GPIO_PULLUP_ENABLE=1 } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE=0, GPIO_PULLDOWN_ENABLE=1 } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE=0, GPIO_INTR_POSEDGE=1, GPIO_INTR_NEGEDGE=2, GPIO_INTR_ANYEDGE=3, GPIO_INTR_LOW_LEVEL=4, GPIO_INTR_HIGH_LEVEL=5 } gpio_int_type_t;

typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;

typedef void (*gpio_isr_t)(void*);
static inline esp_err_t gpio_config(const gpio_config_t *pGPIOConfig){(void)pGPIOConfig;return ESP_OK;}
static inline esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode){(void)gpio_num;(void)mode;return ESP_OK;}
static inline esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level){(void)gpio_num;(void)level;return ESP_OK;}
static inline int gpio_get_level(gpio_num_t gpio_num){(void)gpio_num;return 0;}
static inline esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, int pull){(void)gpio_num;(void)pull;return ESP_OK;}
static inline esp_err_t gpio_reset_pin(gpio_num_t gpio_num){(void)gpio_num;return ESP_OK;}
static inline esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args){(void)gpio_num;(void)isr_handler;(void)args;return ESP_OK;}
static inline esp_err_t gpio_install_isr_service(int intr_alloc_flags){(void)intr_alloc_flags;return ESP_OK;}
#define GPIO_PIN_SEL(x) (1ULL<<(x))
