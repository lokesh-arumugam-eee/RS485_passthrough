#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef uint32_t UBaseType_t;
typedef int BaseType_t;
#define portMAX_DELAY   0xFFFFFFFFUL
#define portTICK_PERIOD_MS  1
#define portTICK_RATE_MS    1
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  pdTRUE
#define pdFAIL  pdFALSE
#define errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY (-1)
