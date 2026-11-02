#pragma once
#include "esp_err.h"
#include "esp_bt_defs.h"
#include <stdint.h>
#include <stdbool.h>

#define ESP_GATTS_REG_EVT               0
#define ESP_GATTS_READ_EVT              1
#define ESP_GATTS_WRITE_EVT             2
#define ESP_GATTS_EXEC_WRITE_EVT        3
#define ESP_GATTS_MTU_EVT               4
#define ESP_GATTS_CONF_EVT              5
#define ESP_GATTS_UNREG_EVT             6
#define ESP_GATTS_CREATE_EVT            7
#define ESP_GATTS_ADD_INCL_SRVC_EVT     8
#define ESP_GATTS_ADD_CHAR_EVT          9
#define ESP_GATTS_ADD_CHAR_DESCR_EVT   10
#define ESP_GATTS_DELETE_EVT           11
#define ESP_GATTS_START_EVT            12
#define ESP_GATTS_STOP_EVT             13
#define ESP_GATTS_CONNECT_EVT          14
#define ESP_GATTS_DISCONNECT_EVT       15
#define ESP_GATTS_OPEN_EVT             16
#define ESP_GATTS_CANCEL_OPEN_EVT      17
#define ESP_GATTS_CLOSE_EVT            18
#define ESP_GATTS_LISTEN_EVT           19
#define ESP_GATTS_CONGEST_EVT          20
#define ESP_GATTS_RESPONSE_EVT         21
#define ESP_GATTS_CREAT_ATTR_TAB_EVT   22
#define ESP_GATTS_SET_ATTR_VAL_EVT     23
#define ESP_GATTS_SEND_SERVICE_CHANGE_EVT 24

typedef int esp_gatts_cb_event_t;

#define ESP_GATT_PERM_READ             (1<<0)
#define ESP_GATT_PERM_READ_ENCRYPTED   (1<<1)
#define ESP_GATT_PERM_WRITE            (1<<4)
#define ESP_GATT_PERM_WRITE_ENCRYPTED  (1<<5)
#define ESP_GATT_CHAR_PROP_BIT_NOTIFY  (1<<4)
#define ESP_GATT_CHAR_PROP_BIT_WRITE_NR (1<<2)
#define ESP_GATT_CHAR_PROP_BIT_READ    (1<<1)
#define ESP_GATT_CHAR_PROP_BIT_INDICATE (1<<5)
#define ESP_GATT_AUTO_RSP              {0}
#define ESP_GATT_RSP_BY_APP            1

#define ESP_GATT_UUID_PRI_SERVICE        0x2800
#define ESP_GATT_UUID_CHAR_DECLARE       0x2803
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG 0x2902

typedef struct {
    uint8_t auto_rsp;
} esp_attr_control_t;

typedef struct {
    uint16_t uuid_length;
    uint8_t  *uuid_p;
    uint16_t perm;
    uint16_t max_length;
    uint16_t length;
    uint8_t  *value;
} esp_attr_desc_t;

typedef struct {
    esp_attr_control_t attr_control;
    esp_attr_desc_t att_desc;
} esp_gatts_attr_db_t;

typedef void (*esp_gatts_cb_t)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, void *param);

extern esp_gatts_cb_t _stub_gatts_cb;
static inline esp_err_t esp_ble_gatts_register_callback(esp_gatts_cb_t callback){_stub_gatts_cb=callback;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_app_register(uint16_t app_id){(void)app_id;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_create_attr_tab(const esp_gatts_attr_db_t *gatts_attr_db, esp_gatt_if_t gatts_if, uint16_t max_nb_attr, uint8_t srvc_inst_id){(void)gatts_attr_db;(void)gatts_if;(void)max_nb_attr;(void)srvc_inst_id;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_start_service(uint16_t service_handle){(void)service_handle;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle, uint16_t value_len, uint8_t *value, bool need_confirm){(void)gatts_if;(void)conn_id;(void)attr_handle;(void)value_len;(void)value;(void)need_confirm;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_send_response(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id, esp_gatt_status_t status, void *rsp){(void)gatts_if;(void)conn_id;(void)trans_id;(void)status;(void)rsp;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_set_attr_value(uint16_t attr_handle, uint16_t length, const uint8_t *value){(void)attr_handle;(void)length;(void)value;return ESP_OK;}
static inline esp_err_t esp_ble_gatts_get_attr_value(uint16_t attr_handle, uint16_t *length, const uint8_t **value){(void)attr_handle;if(length)*length=0;if(value)*value=NULL;return ESP_OK;}
