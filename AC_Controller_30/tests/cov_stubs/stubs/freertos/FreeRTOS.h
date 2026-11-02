#pragma once
#include "portmacro.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define configTICK_RATE_HZ   1000
#define configMAX_PRIORITIES 25
#define tskIDLE_PRIORITY     0
#define configMINIMAL_STACK_SIZE 128

typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* TaskHandle_t;
typedef void* EventGroupHandle_t;
typedef void* TimerHandle_t;
typedef void (*TaskFunction_t)(void*);
typedef TickType_t portTickType;

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
/* Increment each call so retry/timeout loops in firmware code (e.g.
   uart_read_cb) eventually exit instead of spinning forever. */
static inline TickType_t xTaskGetTickCount(void){ static TickType_t _t = 0; return _t++; }
static inline void vTaskDelay(TickType_t xTicksToDelay){(void)xTicksToDelay;}
static inline void vTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement){(void)pxPreviousWakeTime;(void)xTimeIncrement;}
static inline void vTaskSuspend(TaskHandle_t xTaskToSuspend){(void)xTaskToSuspend;}
static inline void vTaskResume(TaskHandle_t xTaskToResume){(void)xTaskToResume;}
static inline void vTaskDelete(TaskHandle_t xTaskToDelete){(void)xTaskToDelete;}
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void){return NULL;}

static inline BaseType_t xTaskCreate(TaskFunction_t pvTaskCode, const char * const pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pvCreatedTask){
    (void)pvTaskCode;(void)pcName;(void)usStackDepth;(void)pvParameters;(void)uxPriority;
    if(pvCreatedTask)*pvCreatedTask=NULL;
    return pdPASS;
}
static inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pvTaskCode, const char * const pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pvCreatedTask, int xCoreID){
    (void)xCoreID;
    return xTaskCreate(pvTaskCode,pcName,usStackDepth,pvParameters,uxPriority,pvCreatedTask);
}
