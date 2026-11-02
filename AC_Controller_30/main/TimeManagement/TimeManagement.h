/**
 ******************************************************************************
 * @file    TimeManagement.h
 * @author  Saiuday Bandaru
 * @brief   This file contains all the functions prototypes for the
 *          different types of sensors.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIME_MNGMT_H
#define TIME_MNGMT_H

/* Includes ------------------------------------------------------------------*/
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"

#include "time.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

//#define ENABLE_TIME_MNGMT 1
void TimeMngmt_init(void);
void set_current_time(time_t ttime_sec);
void LED_init(void);

typedef enum {
  LED_CONTINOUS_ON,
  LED_SLOW_BLINKING,
  LED_FAST_BLINKING,
  LED_BLINK_TWICE,
  LED_BLINK_THRICE,
  LED_LONGON_BLINK,
} led_mode_t;

//extern holiday_t holidays[10];
extern time_t now;
extern struct tm timeinfo;
void LED_Logic(void);
extern  bool time_is_synced;
#endif /*TIME_MNGMT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
