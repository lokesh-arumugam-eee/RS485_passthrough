#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef int uart_port_t;
#define UART_NUM_0  0
#define UART_NUM_1  1
#define UART_NUM_2  2
#define UART_NUM_MAX 3

typedef enum { UART_DATA_5_BITS=0, UART_DATA_6_BITS, UART_DATA_7_BITS, UART_DATA_8_BITS } uart_word_length_t;
typedef enum { UART_STOP_BITS_1=1, UART_STOP_BITS_1_5=2, UART_STOP_BITS_2=3 } uart_stop_bits_t;
typedef enum { UART_PARITY_DISABLE=0, UART_PARITY_EVEN=2, UART_PARITY_ODD=3 } uart_parity_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE=0, UART_HW_FLOWCTRL_RTS=1, UART_HW_FLOWCTRL_CTS=2, UART_HW_FLOWCTRL_CTS_RTS=3 } uart_hw_flowcontrol_t;
typedef enum { UART_SIGNAL_INV_DISABLE=0, UART_SIGNAL_RXD_INV=1, UART_SIGNAL_CTS_INV=2, UART_SIGNAL_DSR_INV=4, UART_SIGNAL_TXD_INV=8, UART_SIGNAL_RTS_INV=16, UART_SIGNAL_DTR_INV=32 } uart_signal_inv_t;
typedef enum { UART_EVENT_DATA=0, UART_EVENT_BREAK, UART_EVENT_BUFFER_FULL, UART_EVENT_FIFO_OVF, UART_EVENT_FRAME_ERR, UART_EVENT_PARITY_ERR, UART_EVENT_DATA_BREAK, UART_EVENT_PATTERN_DET, UART_EVENT_WAKEUP, UART_EVENT_MAX } uart_event_type_t;
typedef enum { UART_SCLK_DEFAULT=0, UART_SCLK_APB=1, UART_SCLK_REF_TICK=2, UART_SCLK_XTAL=3 } uart_sclk_t;
typedef enum { UART_MODE_UART=0, UART_MODE_RS485_HALF_DUPLEX=1, UART_MODE_RS485_COLLISION_DETECT=2, UART_MODE_RS485_APP_CTRL=3, UART_MODE_IRDA=4 } uart_mode_t;

typedef struct {
    uart_event_type_t type;
    size_t size;
    bool timeout_flag;
} uart_event_t;

typedef struct {
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    uint8_t rx_flow_ctrl_thresh;
    uart_sclk_t source_clk;
} uart_config_t;

#include "freertos/FreeRTOS.h"

static inline esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags){(void)uart_num;(void)rx_buffer_size;(void)tx_buffer_size;(void)queue_size;(void)uart_queue;(void)intr_alloc_flags;return ESP_OK;}
static inline esp_err_t uart_driver_delete(uart_port_t uart_num){(void)uart_num;return ESP_OK;}
static inline esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config){(void)uart_num;(void)uart_config;return ESP_OK;}
static inline esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num){(void)uart_num;(void)tx_io_num;(void)rx_io_num;(void)rts_io_num;(void)cts_io_num;return ESP_OK;}
static inline int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait){(void)uart_num;(void)buf;(void)length;(void)ticks_to_wait;return 0;}
static inline int uart_write_bytes(uart_port_t uart_num, const void *src, size_t size){(void)uart_num;(void)src;(void)size;return (int)size;}
static inline esp_err_t uart_flush(uart_port_t uart_num){(void)uart_num;return ESP_OK;}
static inline esp_err_t uart_flush_input(uart_port_t uart_num){(void)uart_num;return ESP_OK;}
static inline esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate){(void)uart_num;(void)baudrate;return ESP_OK;}
static inline esp_err_t uart_get_baudrate(uart_port_t uart_num, uint32_t *baudrate){(void)uart_num;if(baudrate)*baudrate=115200;return ESP_OK;}
static inline esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait){(void)uart_num;(void)ticks_to_wait;return ESP_OK;}
static inline esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t *size){(void)uart_num;if(size)*size=0;return ESP_OK;}
static inline esp_err_t uart_enable_pattern_det_baud_intr(uart_port_t uart_num, char pattern_chr, uint8_t chr_num, int chr_tout, int post_idle, int pre_idle){(void)uart_num;(void)pattern_chr;(void)chr_num;(void)chr_tout;(void)post_idle;(void)pre_idle;return ESP_OK;}
static inline int uart_pattern_pop_pos(uart_port_t uart_num){(void)uart_num;return -1;}
static inline esp_err_t uart_pattern_queue_reset(uart_port_t uart_num, int queue_length){(void)uart_num;(void)queue_length;return ESP_OK;}
static inline esp_err_t uart_set_mode(uart_port_t uart_num, uart_mode_t mode){(void)uart_num;(void)mode;return ESP_OK;}
static inline esp_err_t uart_set_rx_full_threshold(uart_port_t uart_num, int threshold){(void)uart_num;(void)threshold;return ESP_OK;}
static inline esp_err_t uart_set_rx_timeout(uart_port_t uart_num, const uint8_t tout_thresh){(void)uart_num;(void)tout_thresh;return ESP_OK;}
#define UART_PIN_NO_CHANGE -1
#define UART_DATA UART_EVENT_DATA
