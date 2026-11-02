#ifndef UARTAPP_H
#define UARTAPP_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_attr.h"
#include "main.h"

void uart_recive_function(void);
void uart_init(void);
void uart1_init(void);
void IRAM_ATTR uart_isr_handler(void *arg);
#define UART_PORT       UART_NUM_0
#define UART_PORT1     UART_NUM_1
#define BUF_SIZE        1024

#if (MODULE_TYPE == TARGET_ESP32_C3)
#define UART_TX_PIN     22
#define UART_RX_PIN     21

#define UART_TX_PIN1   4
#define UART_RX_PIN1   5
#define RS485_RD_PIN   2

#elif (MODULE_TYPE == TARGET_ESP32_S3)
#define UART_TX_PIN     43
#define UART_RX_PIN     44

#define UART_TX_PIN1   18
#define UART_RX_PIN1   17
#define RS485_RD_PIN   16
#elif (MODULE_TYPE == TARGET_ESP32_S3_WAVESHARE)

#define UART_TX_PIN     15
#define UART_RX_PIN     16

#define UART_TX_PIN1   17
#define UART_RX_PIN1   18
#define RS485_RD_PIN   21

#endif

// ISR buffer
extern uint8_t uart_rx_buffer[BUF_SIZE];
extern volatile int uart_rx_len;
extern volatile bool rx_ready;
extern volatile size_t uart_rx_head;
extern QueueHandle_t uart_queue;

extern volatile uint8_t uart1_rx_buffer[BUF_SIZE];
extern volatile size_t uart1_rx_head ;
extern QueueHandle_t uart1_queue;

#endif /*UART_H */
