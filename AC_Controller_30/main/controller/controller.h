/**
 ******************************************************************************
 * @file    controller.h
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
#ifndef CONTROL_H
#define CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "esp_timer.h"
#include <stdint.h>

#define ACM_SET 0b1
#define ACM_RESET 0b0
// #define AC_CNTRL_EVENT_BASE	"AC_CNTRL_EVENTS" // Define a custom event base

// Event identifiers
typedef enum {
  EVENT_ID_UNKNOWN = 0,
  EVENT_ID_WIFI_CONNECTED,
  EVENT_ID_WIFI_DISCONNECTED,
  EVENT_ID_MQTT_CONNECTED,
  EVENT_ID_MQTT_DISCONNECTED,
  EVENT_ID_MQTT_DATA_RX,
  EVENT_ID_BLE_CONNECTED,
  EVENT_ID_BLE_DISCONNECTED,
  EVENT_ID_BLE_DATA_RX,
  EVENT_ID_UART0_DATA_RX,
  EVENT_ID_UART1_DATA_RX,
  EVENT_ID_USB_DATA_RX,
  EVENT_ID_SCHEDULE_ON,
  EVENT_ID_SCHEDULE_OFF,
  EVENT_ID_MODE,
  EVENT_ID_CSPT,
  EVENT_ID_UART_DATA_RX
  
} custom_event_id_t;

typedef enum{
	SET_ATT=1,    //1
	GET_ATT,      //2
	SET_CONF,     //3
	GET_CONF,     //4
	DEL_CONF,      //5
	REBOOT =102
}E_cmd_states;


typedef enum {
    CMD_UNKNOWN = 0,
    CMD_WIFI_CONFIG,
    CMD_MQTT_CONFIG,
    CMD_WIFI_READ,
    CMD_MQTT_READ,
    CMD_IRTEST,
    CMD_STATUS
} cmd_type_t;


extern void Controller_init(void);

extern uint8_t IR_ON_seq;
extern bool IR_TEST_Completed;
extern uint8_t CMD_Recieved;
extern uint8_t IR_Transmit_blink;
extern bool reboot;
extern uint8_t rebootcnter;
extern volatile uint8_t g_u8_addtlDisblFlgfrlg;
void writesch(void);
ESP_EVENT_DECLARE_BASE(AC_CNTRL_EVENT_BASE);
typedef enum { JIO_ESP_SCHDL_CFG_SET = 2, JIO_ESP_SCHDL_CFG_GET } REV_CMD_e;
 void Jio_ESP_IRBL_Get_Schdl_Inf(char *Payload);
 cmd_type_t get_cmd_type(const char *key);
 void parseMQTT_revCmd(const char *json_string);
 extern esp_timer_handle_t one_shot_timer1;
 extern esp_timer_handle_t cmtimeout_timer1;
 extern  uint8_t g_u8_commtimeout;
 /* BUG-FIX #7: volatile so compiler can't cache reads across loop bodies. */
 extern volatile uint8_t g_u8_acceptFileRx;
 extern volatile uint8_t g_u8_acceptfwRx;
 extern volatile uint8_t g_u8_SendFileXMDMTx;
 extern uint16_t g_u16_expctd_blknm;
 extern uint16_t g_u16_Recieved_blknm;
 extern uint16_t g_u16_nxt_blknm;
 extern uint8_t Test_Mode;
 extern uint8_t DIR_BUT;
 extern uint8_t log_endis;
 extern uint8_t excutelog;
 extern uint8_t l_u8_ack_nak;
#endif /*CONTROL_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
