#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct { uint8_t addr[6]; } esp_bd_addr_t;
typedef uint8_t esp_ble_addr_type_t;
#define BLE_ADDR_TYPE_PUBLIC 0x00
#define BLE_ADDR_TYPE_RANDOM 0x01

typedef struct { uint16_t len; union { uint16_t uuid16; uint32_t uuid32; uint8_t uuid128[16]; } uuid; } esp_bt_uuid_t;
#define ESP_UUID_LEN_16  2
#define ESP_UUID_LEN_32  4
#define ESP_UUID_LEN_128 16

typedef struct { uint8_t *p_data; uint16_t len; } esp_attr_value_t;

typedef uint8_t esp_gatt_if_t;
#define ESP_GATT_IF_NONE  0xFF

typedef enum { ESP_GATT_OK=0, ESP_GATT_INVALID_HANDLE=0x01, ESP_GATT_NO_RESOURCES=0x80, ESP_GATT_INTERNAL_ERROR=0x81, ESP_GATT_WRONG_STATE=0x82, ESP_GATT_DB_FULL=0x83, ESP_GATT_BUSY=0x84, ESP_GATT_ERROR=0x85, ESP_GATT_CMD_STARTED=0x86 } esp_gatt_status_t;

typedef uint16_t esp_gatt_conn_reason_t;
typedef enum { ESP_BT_STATUS_SUCCESS=0, ESP_BT_STATUS_FAIL, ESP_BT_STATUS_NOT_READY, ESP_BT_STATUS_NOMEM, ESP_BT_STATUS_BUSY, ESP_BT_STATUS_DONE, ESP_BT_STATUS_UNSUPPORTED, ESP_BT_STATUS_PARM_INVALID, ESP_BT_STATUS_UNHANDLED, ESP_BT_STATUS_AUTH_FAILURE, ESP_BT_STATUS_RMT_DEV_DOWN, ESP_BT_STATUS_AUTH_REJECTED, ESP_BT_STATUS_INVALID_STATIC_RAND_ADDR, ESP_BT_STATUS_PENDING, ESP_BT_STATUS_UNACCEPT_CONN_INTERVAL, ESP_BT_STATUS_PARAM_OUT_OF_RANGE, ESP_BT_STATUS_TIMEOUT, ESP_BT_STATUS_PEER_LE_DATA_LEN_UNSUPPORTED, ESP_BT_STATUS_CONTROL_LE_DATA_LEN_UNSUPPORTED, ESP_BT_STATUS_ERR_ILLEGAL_PARAMETER_FMT, ESP_BT_STATUS_MEMORY_FULL, ESP_BT_STATUS_EIR_TOO_LARGE } esp_bt_status_t;
typedef uint16_t esp_gatt_perm_t;
typedef uint8_t  esp_gatt_char_prop_t;
typedef struct { esp_bt_uuid_t id; uint8_t is_primary; } esp_gatt_srvc_id_t;

typedef union {
    struct { uint16_t conn_id; uint8_t link_role; esp_bd_addr_t remote_bda; } connect;
    struct { uint16_t conn_id; } disconnect;
    struct { uint16_t handle; uint32_t trans_id; uint16_t conn_id; esp_bd_addr_t bda; bool need_rsp; bool is_long; } read;
    struct { uint16_t handle; uint16_t conn_id; uint32_t trans_id; bool is_prep; uint16_t len; uint8_t *value; } write;
    struct { uint16_t conn_id; bool exec_write_flag; } exec_write;
    struct { uint16_t conn_id; uint16_t mtu; } mtu;
    struct { esp_gatt_status_t status; uint16_t app_id; } reg;
    struct { esp_gatt_status_t status; uint16_t num_handle; uint16_t *handles; } add_attr_tab;
    struct { esp_gatt_status_t status; uint16_t attr_handle; } create;
} esp_ble_gatts_cb_param_t;
