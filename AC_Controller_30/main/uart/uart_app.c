#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "uart_app.h"
#include "configuration/configuration.h"


// RX buffer (volatile because ISR modifies it)
uint8_t uart_rx_buffer[BUF_SIZE];
volatile size_t uart_rx_head = 0;
QueueHandle_t uart_queue;

volatile uint8_t uart1_rx_buffer[BUF_SIZE];
volatile size_t uart1_rx_head = 0;
QueueHandle_t uart1_queue;

// ISR callback for RX


void uart_init(void)
{
	// UART configuration
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, BUF_SIZE, 0, 10, &uart_queue, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart1_init(void)
{

    uart_config_t uart_config1 = {
        .baud_rate = (int)system_urts[0].baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if(system_urts[0].parity == 0)
        uart_config1.parity = UART_PARITY_DISABLE;
    else if(system_urts[0].parity == 1)
        uart_config1.parity = UART_PARITY_ODD;
    else if(system_urts[0].parity == 2)
        uart_config1.parity = UART_PARITY_EVEN;
    else
        uart_config1.parity = UART_PARITY_DISABLE; // Default to no parity if invalid value

    if(system_urts[0].data_bits == 7)
        uart_config1.data_bits = UART_DATA_7_BITS;  
    else if(system_urts[0].data_bits == 8)
        uart_config1.data_bits = UART_DATA_8_BITS;
    else        
        uart_config1.data_bits = UART_DATA_8_BITS; // Default to 8 data bits
    
    if(system_urts[0].stop_bits == 1)
        uart_config1.stop_bits = UART_STOP_BITS_1;  
    else if(system_urts[0].stop_bits == 2)
        uart_config1.stop_bits = UART_STOP_BITS_2;
    else
        uart_config1.stop_bits = UART_STOP_BITS_1; // Default to 1 stop bit 

    // 1. Install driver (Set TX and RX buffer sizes)
    // Using 1024 as a standard BUF_SIZE
    uart_driver_install(UART_NUM_1, 1024, 1024, 10, &uart1_queue, 0);

    // 2. Configure UART parameters
    uart_param_config(UART_NUM_1, &uart_config1);

    // 3. Set Pins (Ensure RS485_RD_PIN is your RE/DE pin)
    uart_set_pin(UART_NUM_1, UART_TX_PIN1, UART_RX_PIN1, RS485_RD_PIN, UART_PIN_NO_CHANGE);
    printf("TXD PIN :%d,RXD PIN %d,DIR PIN :%d",UART_TX_PIN1,UART_RX_PIN1,RS485_RD_PIN);

    // 4. Set RS485 Half Duplex Mode
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);

    // 5. The "Inter-byte Delay" setup
    // Instead of uart_intr_config, use these two standard functions:
    
    // Trigger an interrupt when 100 bytes are in the buffer (High threshold)
    uart_set_rx_full_threshold(UART_NUM_1, 100);
    
    // TRIGGER an interrupt if the line is idle for 10 symbols (at 9600 baud, ~10ms)
    // This is what will make your data come in as one "chunk"
    uart_set_rx_timeout(UART_NUM_1, 10);
}
