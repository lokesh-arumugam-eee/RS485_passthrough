#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

typedef int i2c_port_t;
#define I2C_NUM_0 0
#define I2C_NUM_1 1
#define I2C_NUM_MAX 2

typedef enum { I2C_MODE_SLAVE=0, I2C_MODE_MASTER } i2c_mode_t;
typedef void* i2c_cmd_handle_t;

typedef struct {
    i2c_mode_t mode;
    int sda_io_num;
    int scl_io_num;
    bool sda_pullup_en;
    bool scl_pullup_en;
    union {
        struct { uint32_t clk_speed; } master;
        struct { uint8_t addr_10bit_en; uint16_t slave_addr; uint32_t maximum_speed; } slave;
    };
    uint32_t clk_flags;
} i2c_config_t;

static inline esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t *i2c_conf){(void)i2c_num;(void)i2c_conf;return ESP_OK;}
static inline esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags){(void)i2c_num;(void)mode;(void)slv_rx_buf_len;(void)slv_tx_buf_len;(void)intr_alloc_flags;return ESP_OK;}
static inline i2c_cmd_handle_t i2c_cmd_link_create(void){return (i2c_cmd_handle_t)malloc(1);}
static inline void i2c_cmd_link_delete(i2c_cmd_handle_t cmd_handle){free(cmd_handle);}
static inline esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle){(void)cmd_handle;return ESP_OK;}
static inline esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle){(void)cmd_handle;return ESP_OK;}
static inline esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en){(void)cmd_handle;(void)data;(void)ack_en;return ESP_OK;}
static inline esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, const uint8_t *data, size_t data_len, bool ack_en){(void)cmd_handle;(void)data;(void)data_len;(void)ack_en;return ESP_OK;}
static inline esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, int ack){(void)cmd_handle;if(data)*data=0;(void)ack;return ESP_OK;}
static inline esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, int ack){(void)cmd_handle;if(data)memset(data,0,data_len);(void)ack;return ESP_OK;}
static inline esp_err_t i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait){(void)i2c_num;(void)cmd_handle;(void)ticks_to_wait;return ESP_OK;}
static inline esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer, size_t write_size, TickType_t ticks_to_wait){(void)i2c_num;(void)device_address;(void)write_buffer;(void)write_size;(void)ticks_to_wait;return ESP_OK;}
static inline esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address, uint8_t *read_buffer, size_t read_size, TickType_t ticks_to_wait){(void)i2c_num;(void)device_address;if(read_buffer)memset(read_buffer,0,read_size);(void)ticks_to_wait;return ESP_OK;}
static inline esp_err_t i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, TickType_t ticks_to_wait){(void)i2c_num;(void)device_address;(void)write_buffer;(void)write_size;if(read_buffer)memset(read_buffer,0,read_size);(void)ticks_to_wait;return ESP_OK;}
#define I2C_MASTER_ACK  0
#define I2C_MASTER_NACK 1
#define I2C_MASTER_LAST_NACK 2
#define I2C_MASTER_READ  1
#define I2C_MASTER_WRITE 0
#include <string.h>
