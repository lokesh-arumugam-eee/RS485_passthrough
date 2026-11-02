#pragma once
#include "esp_err.h"
typedef struct {} esp_vfs_t;
static inline esp_err_t esp_vfs_dev_uart_register(void){return ESP_OK;}
static inline void esp_vfs_dev_uart_use_driver(int uart_num){(void)uart_num;}
static inline void esp_vfs_dev_uart_use_nonblocking(int uart_num){(void)uart_num;}
static inline void esp_vfs_dev_uart_port_set_rx_line_endings(int uart_num, int mode){(void)uart_num;(void)mode;}
static inline void esp_vfs_dev_uart_port_set_tx_line_endings(int uart_num, int mode){(void)uart_num;(void)mode;}
