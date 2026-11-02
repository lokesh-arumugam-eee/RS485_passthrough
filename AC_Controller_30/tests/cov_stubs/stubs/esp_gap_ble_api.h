#pragma once
#include "esp_err.h"
#include "esp_bt_defs.h"
#include <stdint.h>
#include <stdbool.h>

#define ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT        0
#define ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT   1
#define ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT      2
#define ESP_GAP_BLE_SCAN_RESULT_EVT                  3
#define ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT    4
#define ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT 5
#define ESP_GAP_BLE_ADV_START_COMPLETE_EVT           6
#define ESP_GAP_BLE_SCAN_START_COMPLETE_EVT          7

typedef int esp_gap_ble_cb_event_t;
typedef int esp_ble_adv_channel_t;
typedef int esp_ble_adv_filter_t;
typedef int esp_ble_adv_type_t;

#define ADV_TYPE_IND            0x00
#define ADV_CHNL_ALL            0x07
#define ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY 0x00

typedef struct {
    uint8_t adv_type;
    uint8_t own_addr_type;
    uint8_t channel_map;
    uint8_t adv_filter_policy;
    uint8_t peer_addr_type;
    esp_bd_addr_t peer_addr;
    uint16_t adv_int_min;
    uint16_t adv_int_max;
} esp_ble_adv_params_t;

typedef struct {
    bool set_scan_rsp;
    bool include_name;
    bool include_txpower;
    int min_interval;
    int max_interval;
    int appearance;
    uint16_t manufacturer_len;
    uint8_t *p_manufacturer_data;
    uint16_t service_data_len;
    uint8_t *p_service_data;
    uint16_t service_uuid_len;
    uint8_t *p_service_uuid;
    uint8_t flag;
} esp_ble_adv_data_t;

typedef union {
    struct { esp_gatt_status_t status; } adv_data_cmpl;
    struct { esp_gatt_status_t status; } scan_rsp_data_cmpl;
    struct { esp_gatt_status_t status; } adv_start_cmpl;
} esp_ble_gap_cb_param_t;

typedef void (*esp_gap_ble_cb_t)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* Coverage harness: remember the registered callback so tests can dispatch
   GAP events to the static gap_event_handler in firmware. */
extern esp_gap_ble_cb_t _stub_gap_cb;
static inline esp_err_t esp_ble_gap_register_callback(esp_gap_ble_cb_t callback){_stub_gap_cb=callback;return ESP_OK;}
static inline esp_err_t esp_ble_gap_config_adv_data(esp_ble_adv_data_t *adv_data){(void)adv_data;return ESP_OK;}
static inline esp_err_t esp_ble_gap_set_device_name(const char *name){(void)name;return ESP_OK;}
static inline esp_err_t esp_ble_gap_start_advertising(esp_ble_adv_params_t *adv_params){(void)adv_params;return ESP_OK;}
static inline esp_err_t esp_ble_gap_stop_advertising(void){return ESP_OK;}
static inline esp_err_t esp_ble_gap_config_adv_data_raw(uint8_t *raw_data, uint32_t raw_data_len){(void)raw_data;(void)raw_data_len;return ESP_OK;}
static inline esp_err_t esp_ble_gap_config_scan_rsp_data_raw(uint8_t *raw_data, uint32_t raw_data_len){(void)raw_data;(void)raw_data_len;return ESP_OK;}
static inline esp_err_t esp_ble_gap_disconnect(esp_bd_addr_t remote_device){(void)remote_device;return ESP_OK;}
static inline esp_err_t esp_ble_gap_update_conn_params(void *params){(void)params;return ESP_OK;}
