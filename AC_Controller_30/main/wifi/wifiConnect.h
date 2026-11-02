/**
 ******************************************************************************
 * @file    mqtt.h
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
#ifndef MQTT_H
#define MQTT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "esp_netif.h" 
#include <stdint.h>

#define SSID_MAX_LEN 32
#define WIFI_PASS_MAX_LEN 64

typedef struct {
  uint8_t ssid[SSID_MAX_LEN];     /**< SSID of target AP. */
  uint8_t password[WIFI_PASS_MAX_LEN]; /**< Password of target AP. */
  uint8_t RtryC;        // Connection Retry count
  uint8_t Key;          // Key to verify NVS data default=0xAA
} WIFICONFIG_S;
extern WIFICONFIG_S wifiConfig;

/* Private includes ----------------------------------------------------------*/
void wifi_init_sta(void);
/*	ReadWriteNVS_wifiConfig function
parameters
Type 			-> 0=Read WifiConfig into NVS, 1=Write WifiConfig into
NVS, 2=Write Config and Set wifi Station to new Configuration
WIFICONFIG_S*	-> Pointer to new configuration.
*/
extern bool wifi_is_connected;
extern esp_netif_ip_info_t ip_info;
extern bool static_ip_enabled;
esp_err_t ReadWriteNVS_wifiConfig(uint8_t Type, WIFICONFIG_S *wifiConfig_t);
esp_err_t ReadWriteNVS(uint8_t Type, uint8_t *data, char *key, int length);
esp_err_t change_wifi_credentials(const char *new_ssid,const char *new_password); 
#endif /*MQTT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
