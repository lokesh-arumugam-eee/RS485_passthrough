#pragma once
#include "FreeRTOS.h"

typedef void* EventGroupHandle_t;
typedef uint32_t EventBits_t;

static inline EventGroupHandle_t xEventGroupCreate(void){return (EventGroupHandle_t)malloc(sizeof(uint32_t));}
static inline void vEventGroupDelete(EventGroupHandle_t xEventGroup){free(xEventGroup);}
static inline EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet){(void)xEventGroup;return uxBitsToSet;}
static inline EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear){(void)xEventGroup;(void)uxBitsToClear;return 0;}
static inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait){(void)xEventGroup;(void)xClearOnExit;(void)xWaitForAllBits;(void)xTicksToWait;return uxBitsToWaitFor;}
static inline EventBits_t xEventGroupGetBits(EventGroupHandle_t xEventGroup){(void)xEventGroup;return 0;}
#define xEventGroupSetBitsFromISR(eg,b,w) xEventGroupSetBits(eg,b)
