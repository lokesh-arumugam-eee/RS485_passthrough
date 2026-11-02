#pragma once
#include "FreeRTOS.h"

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void){return (SemaphoreHandle_t)malloc(1);}
static inline SemaphoreHandle_t xSemaphoreCreateBinary(void){return (SemaphoreHandle_t)malloc(1);}
static inline SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount){(void)uxMaxCount;(void)uxInitialCount;return (SemaphoreHandle_t)malloc(1);}
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore){(void)xSemaphore;return pdTRUE;}
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime){(void)xSemaphore;(void)xBlockTime;return pdTRUE;}
static inline void vSemaphoreDelete(SemaphoreHandle_t xSemaphore){free(xSemaphore);}
#define xSemaphoreGiveFromISR(s,w) (pdTRUE)
#define xSemaphoreTakeFromISR(s,w) (pdTRUE)
