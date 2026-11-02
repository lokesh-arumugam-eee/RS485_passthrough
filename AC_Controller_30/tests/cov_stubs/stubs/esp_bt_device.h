#pragma once
#include "esp_err.h"
#include "esp_bt_defs.h"
#include <stdint.h>
#include <stdbool.h>
static inline const uint8_t *esp_bt_dev_get_address(void){static uint8_t addr[6]={0};return addr;}
static inline esp_err_t esp_bt_dev_set_device_name(const char *name){(void)name;return ESP_OK;}
