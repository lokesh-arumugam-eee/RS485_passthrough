#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nanomodbus.h"
#include <sys/socket.h>
#include "esp_netif.h"

typedef struct {
    int connfd;
} modbus_client_ctx_t;

typedef enum {
    READ_HOLDING_REGISTERS = 1,
    READ_INPUT_REGISTERS = 2,
    WRITE_MULTIPLE_REGISTERS = 3
} tcp_request_type_t;

#define SLAVE_ID  249 // Reserved for special filename/IP/Baudrate logic
extern void tcp_app_init(void);
nmbs_error modbus_multi_server_read(uint8_t slave_id, esp_ip4_addr_t poll_ip,int port, uint16_t address, int quantity, uint16_t* registers_out);


#endif