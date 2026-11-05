#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nanomodbus.h"
#include "modbus_tcp.h"


#define ADD_FW_VER	0
#define ADD_REL_DATE	2
#define ADD_WIFI_MAC	4
#define ADD_BLE_MAC	7
#define ADD_WIFI_RSSI	9	// RO: signed dBm, 127 = not connected
#define ADD_BAUDRATE	10
#define ADD_PARITY	12
#define ADD_DATABITS	13
#define ADD_STOP_BITS	14
#define ADD_RETRY_CNT	15
#define ADD_RETRY_PRD	16
#define ADD_RESERVED_17	17	// formerly RTU swap-CRC; reads 0, writes ignored
#define ADD_STATIC_IP	18
#define ADD_GW_IP	20
#define ADD_NW_MASK	22
#define ADD_TCP_MB_PORT	24
#define ADD_TCP_PT_PORT	25
#define ADD_HTTP_LISTEN_PORT	26
#define ADD_EN_STATIC_IP	27
#define ADD_SRVR_TIMEOUT	28
#define ADD_OTA_HTTP_URL	29
#define ADD_OTA_HTTP_PORT	93
#define ADD_OTA_HTTP_FILE_NAME	94
#define ADD_TRIGGER_FOTA	126
#define ADD_FOTA_STS	127
#define ADD_FOTA_PROGRESS	128
#define ADD_FOTA_FAILURE_REASON	129
#define ADD_PASSTHR_RETRY_CNT	130
/* Debug-log capture controls (mirrors log_params_t in log_capture.h). */
#define ADD_LOG_TRIGGER		131   /* RW: 0=idle, 1=start (auto-cleared by task)   */
#define ADD_LOG_DURATION	132   /* RW: capture window in seconds (0 -> 300)     */
#define ADD_LOG_STATUS		133   /* RO: LOG_STATUS_* (0..4)                       */
#define ADD_LOG_PATH		134   /* RW: 32 regs / 64 bytes URL-path prefix       */
# define CONFIG_MAX_MODBUS_ADDRESS	166
#define ADD_DELETE_FILE	566
#define ADD_REBOOT_DEVICE	567


extern nmbs_t nmbs_rtu_master;
extern nmbs_platform_conf rtu_platform_conf;
extern int current_slave_id;

void modbus_app_init(void);
extern int32_t uart_read_cb(uint8_t* buf, uint16_t count,int32_t timeout_ms, void* arg);
extern int32_t uart_write_cb(const uint8_t* buf, uint16_t count,
                             int32_t timeout_ms, void* arg);
extern  void uart_flush_cb(nmbs_t* nmbs_rtu_master, void* arg);
extern uint16_t modbus_crc16(const uint8_t* data, uint32_t length, void* arg);
extern nmbs_error safe_modbus_read(tcp_request_type_t request_type, uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf);
extern nmbs_error populate_modbus_read_holding(uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf);
extern nmbs_error populate_modbus_read_input(uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf);
extern nmbs_error populate_modbus_write(uint8_t slave_id, uint16_t addr, uint16_t qty, const uint16_t* in_buf);
int run_polling_cycle(void);
bool is_time_to_poll(int mbr_index);
//extern QueueHandle_t uart1_queue;

#endif
