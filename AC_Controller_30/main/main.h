/**
 ******************************************************************************
 * @file    main.h
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
#ifndef AC_MAIN_H
#define AC_MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "string.h"
#include "time.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "SafeFunctions/Jio_EA_Safe_Functions.h"
// Module Types
#define TARGET_ESP32_S3 01
#define TARGET_ESP32_C3 02
#define TARGET_ESP32_S3_WAVESHARE 03
#define MODULE_TYPE TARGET_ESP32_S3

#define JIO_MQTT 01
#define TRUEPOWER_MQTT 02
#define THINGSBOARD_MQTT 03

#define MQTT_DETAILS TRUEPOWER_MQTT

#define FW_VERHIGH 3
#define FW_VERLOW 1
#define FW_VERPATCH 14
#define REL_DA 28
#define REL_MO 07
#define REL_YR 2026

#define HTTPS_CONNECTION 2


#define ACM_TaskSleep_mSec(x) vTaskDelay(x / portTICK_PERIOD_MS);
void set_current_time(time_t ttime_sec);
extern uint8_t mac_ble[6];
extern uint8_t mac_Wifi[6];
extern uint8_t frequency;
extern uint8_t irlenlim;
extern uart_event_t event;
extern char version[16];
extern char release_date[16];
extern char     ota_report_ip[64];
extern uint16_t ota_report_port;
extern char     ota_report_path[64];
#define MAX_LOG_LENGTH 2000


#endif /*AC_MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
