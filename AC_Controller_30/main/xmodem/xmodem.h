/**
 ******************************************************************************
 * @file    xmodem.h
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
#ifndef LIR_H
#define LIR_H

/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include "ctype.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "main.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
// #include "driver/ledc.h"

#define LIR_LOG_TAG "ACM_LIR"


#define MAX_ASSERT_NAME_LEN 12
#define BUT_NAME_LEN 12
#define FRQ_N_LEN_OFFSET 4
#define MAX_BUTTONS 16
#define MAX_ASSERTS 1
#define LEARN_MODE_TX_RX_BUFF_LEN 1536
#define LEARN_MODE_TX_BUFF_LEN LEARN_MODE_TX_RX_BUFF_LEN/2
#define LEARN_MODE_KEY 236 // supports 1-255

extern const char *LIR_CMDs_file_path;
extern const char *tmp_file_path;
extern uint8_t reboot_after_10Sec;
extern uint8_t g_u8_FileSystemErr;


//extern const char *LIR_CMDs_file_path;
extern const char *configpath;
extern void LIR_task_init(void);
// Define a structure for Button Status

extern void Custom_printf(const char *format, ...);
extern void acm_log(const char *format, ...);
esp_err_t delete_file_example(void);
#define JIO_ESP_X_SOH   0x01
#define JIO_ESP_X_EOT   0x04
#define JIO_ESP_X_ACK   0x06
#define JIO_ESP_X_NAK   0x15
#define JIO_ESP_X_CAN   0x18
#define JIO_ESP_X_CRC   'C'

#define JIO_ESP_X_BLOCK_SIZE 500
#define FILE_BUFFER_SIZE 5000// 10 KB
#define FILE_COMMIT_SIZE 4096
//extern uint8_t g_u8_file_buffer[FILE_BUFFER_SIZE];

#define JIO_ESP_SUCCESS 0
#define JIO_ESP_FAILURE -1
typedef struct {
    uint8_t header;
    uint16_t block_num;
    //uint8_t block_num_neg;
    uint8_t data[JIO_ESP_X_BLOCK_SIZE];
    uint8_t checksum;
} __attribute__((packed)) xmodem_packet_t;

typedef enum { // for buttons
  JIO_ESP_XMDM_FSM_INIT = 0,
  JIO_ESP_XMDM_FSM_COMMIT_FILE,
  JIO_ESP_XMDM_RX_FILE_COMPLETE,
  JIO_ESP_XMDM_FSM_READ_SEND_FILE,
  JIO_ESP_XMDM_TX_FILE_COMPLETE,
  JIO_ESP_XMDM_STATE_MAX, // 16
} Xmdm_Rx_FSM_States_e;
void change_log_tag(const char *new_tag);
extern int8_t Jio_ESP_010101_Chk4Xmdm_pkt(uint8_t* );
extern int8_t Jio_ESP_010102_LIRFile_Rx_FSM(uint8_t*);
extern void Jio_ESP_010104_LIRFile_Tx_FSM(uint8_t *);
extern void Jio_ESP_010103_Send_ACK_NAK(uint8_t);
void write_with_resume(const char *data,uint16_t len);
int8_t ReadFromFile(void *Buff, size_t BuffSize, const char *FileName);
int8_t WriteIntoFile(void *Buff, size_t BuffSize, const char *FileName);
int8_t find_string_in_file(const char *file_path, const char *search_text,void *Buff,void *Buff1);
int8_t ReadFromFileOffset(void *Buff, size_t BuffSize, const char *FileName, size_t Offset);
int8_t WriteIntoFileAppend(void *Buff, size_t BuffSize, const char *FileName);
void UpdateSTS(uint8_t but);
esp_err_t spiffs_repair_if_needed(void);
extern uint8_t g_u8_Xmdm_Rx_FSM_STATE;
extern uint8_t g_u8_Xmdm_Tx_FSM_STATE;
extern FILE *g_file;
extern uint8_t g_u8_file_buffer[FILE_BUFFER_SIZE];
extern const char *XMODEM;

#define BULK_CHUNK_COMMIT
#endif /*LIR_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
