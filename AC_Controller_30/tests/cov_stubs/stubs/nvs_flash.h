#pragma once
#include "esp_err.h"
#define ESP_ERR_NVS_NO_FREE_PAGES    0x1101
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1102
#define ESP_ERR_NVS_NOT_ENOUGH_SPACE 0x1103
static inline esp_err_t nvs_flash_init(void){return ESP_OK;}
static inline esp_err_t nvs_flash_init_partition(const char* partition_label){(void)partition_label;return ESP_OK;}
static inline esp_err_t nvs_flash_erase(void){return ESP_OK;}
static inline esp_err_t nvs_flash_erase_partition(const char* partition_label){(void)partition_label;return ESP_OK;}
static inline esp_err_t nvs_flash_deinit(void){return ESP_OK;}
