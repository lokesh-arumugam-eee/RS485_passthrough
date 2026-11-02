#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int controller_task_stack_size;
    int controller_task_prio;
    int hci_uart_no;
    int hci_uart_baudrate;
    int scan_duplicate_mode;
    int scan_duplicate_type;
    int normal_adv_size;
    int mesh_adv_size;
    int send_adv_reserved_size;
    int controller_debug_flag;
    uint8_t mode;
    uint8_t ble_max_conn;
    uint8_t bt_max_acl_conn;
    uint8_t bt_sco_datapath;
    bool auto_latency;
    bool bt_legacy_auth_vs_evt;
    uint8_t bt_max_sync_conn;
    uint8_t ble_sca;
    uint8_t pcm_role;
    uint8_t pcm_polar;
    bool hli;
    uint16_t dup_list_refresh_period;
    bool magic;
} esp_bt_controller_config_t;

#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {0}
#define ESP_BT_MODE_IDLE       0x00
#define ESP_BT_MODE_BLE        0x01
#define ESP_BT_MODE_CLASSIC_BT 0x02
#define ESP_BT_MODE_BTDM       0x03

static inline esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg){(void)cfg;return ESP_OK;}
static inline esp_err_t esp_bt_controller_enable(uint8_t mode){(void)mode;return ESP_OK;}
static inline esp_err_t esp_bt_controller_disable(void){return ESP_OK;}
static inline esp_err_t esp_bt_controller_deinit(void){return ESP_OK;}
static inline esp_err_t esp_bt_mem_release(uint8_t mode){(void)mode;return ESP_OK;}
static inline esp_err_t esp_bt_controller_mem_release(uint8_t mode){(void)mode;return ESP_OK;}
