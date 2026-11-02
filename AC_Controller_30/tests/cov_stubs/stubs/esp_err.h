#pragma once
#include <stdint.h>
#include "esp_bit_defs.h"
typedef int32_t esp_err_t;
#define ESP_OK            0
#define ESP_FAIL         -1
#define ESP_ERR_NO_MEM   0x101
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT   0x107
#define ESP_ERR_INVALID_CRC 0x108
#define ESP_ERR_NVS_NOT_FOUND 0x1100
#define ESP_ERROR_CHECK(x) (x)
#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) (x)
static inline const char* esp_err_to_name(esp_err_t e){(void)e;return "ERR";}
