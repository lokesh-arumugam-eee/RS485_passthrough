#include "main.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "string.h"
#include <stdio.h>
#include "ble_spp_server_demo.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_system.h"
#include "xmodem/xmodem.h"
#include "controller/controller.h"
#include "esp_mac.h"
//#if ENABLE_TIME_MNGMT
#include "TimeManagement.h"
#define GATTS_TABLE_TAG "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME                                                     \
  "IR_Blaster_01" // The Device Name Characteristics in GAP"AC_MONITOR_01"
#define SPP_SVC_INST_ID 0

char device_name[32];

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2
// Add to adv payload

uint8_t spp_adv_data[31];
size_t idx;
void ble_build_adv_data(uint8_t *spp_adv_data, size_t *idx);
static uint16_t spp_mtu_size = 500;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_sendData_queue = NULL;

static bool enable_data_ntf = false;
bool ble_is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x320,
    .adv_int_max = 0x640,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);
/* One gatt-based profile one app_id and one gatts_if, this array will store the
 * gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is
                                             ESP_GATT_IF_NONE */
        },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid =
    ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = {
    // SPP -  Service Declaration
    [SPP_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                      ESP_GATT_PERM_READ, sizeof(spp_service_uuid),
                      sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    // SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16,
                                     (uint8_t *)&character_declaration_uuid,
                                     ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                                     CHAR_DECLARATION_SIZE,
                                     (uint8_t *)&char_prop_read_write}},

    // SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN,
          sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    // SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
          (uint8_t *)&char_prop_read_notify}},

    // SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL] = {{ESP_GATT_AUTO_RSP},
                                  {ESP_UUID_LEN_16,
                                   (uint8_t *)&spp_data_notify_uuid,
                                   ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN,
                                   sizeof(spp_data_notify_val),
                                   (uint8_t *)spp_data_notify_val}},

    // SPP -  data notify characteristic - Client Characteristic Configuration
    // Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
          sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},
};


// Build adv packet with IRBLASTER_<MAC> name
void ble_build_adv_data(uint8_t *adv_raw_data, size_t *adv_len)
{

    // Get BLE MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    //esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // 🔹 Option 1: Short MAC (last 2 bytes)
    char device_name[20];
    sprintf_j(device_name, sizeof(device_name), "IRBLASTER_%02X%02X", mac[4], mac[5]);

    ESP_LOGI("BLE", "Adv Name: %s", device_name);

    // ---- Build adv data ----

    // Flags
    spp_adv_data[idx++] = 0x02;   // Length
    spp_adv_data[idx++] = 0x01;   // Type: Flags
    spp_adv_data[idx++] = 0x06;   // LE General Disc + BR/EDR Not Supported

    // Complete 16-bit UUID (UUID = 0xABF0)
    spp_adv_data[idx++] = 0x03;   // Length
    spp_adv_data[idx++] = 0x03;   // Type: 16-bit UUID
    spp_adv_data[idx++] = 0xF0;   // LSB
    spp_adv_data[idx++] = 0xAB;   // MSB

    // Complete Local Name
    int name_len = strlen(device_name);
    spp_adv_data[idx++] = name_len + 1;  // Length = type + string
    spp_adv_data[idx++] = 0x09;          // Type: Complete Local Name
    //memcpy(&spp_adv_data[idx], device_name, name_len);
    memcpy_j(&spp_adv_data[idx], sizeof(spp_adv_data), device_name, name_len);
    idx += name_len;

    // Load adv data
    //esp_ble_gap_config_adv_data_raw(spp_adv_data, idx);
}
static uint8_t find_char_and_desr_index(uint16_t handle) {
  uint8_t error = 0xff;
  for (int i = 0; i < SPP_IDX_NB; i++) {
    if (handle == spp_handle_table[i]) {
      return i;
    }
  }
  return error;
}
uint8_t ble_spp_SendData(size_t size, uint8_t *temp) {
  if ((size) && (ble_is_connected)) {
    if (!enable_data_ntf) {
      ESP_LOGE(GATTS_TABLE_TAG, "Enable Notify");
      return 0;
    }
    if (size <= (spp_mtu_size - 3)) {
      esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                                  spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                                  size, temp, false);
    } else if (size > (spp_mtu_size - 3)) {
      uint8_t total_num = 0;
      uint8_t current_num = 0;
      if ((size % (spp_mtu_size - 7)) == 0) {
        total_num = size / (spp_mtu_size - 7);
      } else {
        total_num = size / (spp_mtu_size - 7) + 1;
      }
      uint8_t *ntf_value_p = NULL;
      current_num = 1;
      ntf_value_p = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
      if (ntf_value_p == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.2 failed", __func__);
        // free(temp);
        return 0; // fail
      }
      while (current_num <= total_num) {
        if (current_num < total_num) {
          ntf_value_p[0] = '#';
          ntf_value_p[1] = '#';
          ntf_value_p[2] = total_num;
          ntf_value_p[3] = current_num;
          memcpy_j(ntf_value_p + 4, sizeof(ntf_value_p) - 4, temp + (current_num - 1) * (spp_mtu_size - 7),
                 (spp_mtu_size - 7));
          esp_ble_gatts_send_indicate(
              spp_gatts_if, spp_conn_id,
              spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], (spp_mtu_size - 3),
              ntf_value_p, false);
        } else if (current_num == total_num) {
          ntf_value_p[0] = '#';
          ntf_value_p[1] = '#';
          ntf_value_p[2] = total_num;
          ntf_value_p[3] = current_num;
          memcpy_j(ntf_value_p + 4, sizeof(ntf_value_p) - 4, temp + (current_num - 1) * (spp_mtu_size - 7),
                 (size - (current_num - 1) * (spp_mtu_size - 7)));
          esp_ble_gatts_send_indicate(
              spp_gatts_if, spp_conn_id,
              spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
              (size - (current_num - 1) * (spp_mtu_size - 7) + 4), ntf_value_p,
              false);
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
        current_num++;
      }
      free(ntf_value_p);
    }
    return 1; // success
  } else {
    return 0;
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  esp_err_t err;
  ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    
    esp_ble_gap_start_advertising(&spp_adv_params);
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start
    // successfully or failed
    if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(GATTS_TABLE_TAG, "Adv start failed: %s",
               esp_err_to_name(err));
    }
    break;
  default:
    break;
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
  uint8_t res = 0xff;
  if (ble_is_connected != true)
    ESP_LOGI(GATTS_TABLE_TAG, "event = %x", event);
  switch (event) {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
    esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
    esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data,
                                    idx);

    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
    esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB,
                                  SPP_SVC_INST_ID);
    break;
  case ESP_GATTS_READ_EVT:
    res = find_char_and_desr_index(p_data->read.handle);
    if (res == SPP_IDX_SPP_STATUS_VAL) {
      // TODO:client read the status characteristic
    }
    break;
  case ESP_GATTS_WRITE_EVT: {
    res = find_char_and_desr_index(p_data->write.handle);
    if (p_data->write.is_prep == false) {
      if (ble_is_connected != true)
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
      if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
        if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) &&
            (p_data->write.value[1] == 0x00)) {
          enable_data_ntf = true;
        } else if ((p_data->write.len == 2) &&
                   (p_data->write.value[0] == 0x00) &&
                   (p_data->write.value[1] == 0x00)) {
          enable_data_ntf = false;
        }
      } else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
#ifdef SPP_DEBUG_MODE
        esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(p_data->write.value),
                            p_data->write.len);
#else
        // Post EVENT_ID_HELLO with a string message
/*        ESP_EVENT_DECLARE_BASE(AC_CNTRL_EVENT_BASE);
        if(p_data->write.value>EVENT_ID_SCHEDULE_OFF)
        {
			esp_event_post(AC_CNTRL_EVENT_BASE, EVENT_ID_SCHEDULE_ON, NULL, 0 ,portMAX_DELAY);
			vTaskDelay(50 / portTICK_PERIOD_MS);
        }*/
        esp_event_post(AC_CNTRL_EVENT_BASE, EVENT_ID_BLE_DATA_RX,
                       p_data->write.value, p_data->write.len + 1, 0);
        break;

#endif
      } else {
        // TODO:
      }
    } else if ((p_data->write.is_prep == true) &&
               (res == SPP_IDX_SPP_DATA_RECV_VAL)) {
      ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
      // store_wr_buffer(p_data);
    }
    break;
  }
  case ESP_GATTS_EXEC_WRITE_EVT: {
    // if(ble_is_connected!=true)
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    if (p_data->exec_write.exec_write_flag) {
      // print_write_buffer();
      // free_write_buffer();
    }
    break;
  }
  case ESP_GATTS_MTU_EVT:
    spp_mtu_size = p_data->mtu.mtu;
    break;
  case ESP_GATTS_CONF_EVT:
    break;
  case ESP_GATTS_UNREG_EVT:
    break;
  case ESP_GATTS_DELETE_EVT:
    break;
  case ESP_GATTS_START_EVT:
    break;
  case ESP_GATTS_STOP_EVT:
    break;
  case ESP_GATTS_CONNECT_EVT:
    spp_conn_id = p_data->connect.conn_id;
    spp_gatts_if = gatts_if;
    ble_is_connected = true;
    memcpy_j(&spp_remote_bda, sizeof(spp_remote_bda), (const void*)&p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    spp_mtu_size = 23;
    ble_is_connected = false;
    enable_data_ntf = false;
    esp_ble_gap_start_advertising(&spp_adv_params);
    break;
  case ESP_GATTS_OPEN_EVT:
    break;
  case ESP_GATTS_CANCEL_OPEN_EVT:
    break;
  case ESP_GATTS_CLOSE_EVT:
    break;
  case ESP_GATTS_LISTEN_EVT:
    break;
  case ESP_GATTS_CONGEST_EVT:
    break;
  case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x",
             param->add_attr_tab.num_handle);
    if (param->add_attr_tab.status != ESP_GATT_OK) {
      ESP_LOGE(GATTS_TABLE_TAG,
               "Create attribute table failed, error code=0x%x",
               param->add_attr_tab.status);
    } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
      ESP_LOGE(GATTS_TABLE_TAG,
               "Create attribute table abnormally, num_handle (%d) doesn't "
               "equal to HRS_IDX_NB(%d)",
               param->add_attr_tab.num_handle, SPP_IDX_NB);
    } else {
      memcpy_j(spp_handle_table, sizeof(spp_handle_table), param->add_attr_tab.handles,
             sizeof(spp_handle_table));
      esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    }
    break;
  }
  default:
    break;
  }
}
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  if (ble_is_connected != true)
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d", event, gatts_if);

  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",
               param->reg.app_id, param->reg.status);
      return;
    }
  }

  do {
    int idx;
    for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a
                                             certain gatt_if, need to call every
                                             profile cb function */
          gatts_if == spp_profile_tab[idx].gatts_if) {
        if (spp_profile_tab[idx].gatts_cb) {
          spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void ble_gap_gatt_spp_init(void) {

  esp_err_t ret;
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ble_build_adv_data(spp_adv_data, &idx);  

  /* // Initialize NVS
   ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
   { ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK( ret );*/
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(GATTS_TABLE_TAG, "%s init ble", __func__);
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "%s init ble failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable ble failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  esp_ble_gatts_register_callback(gatts_event_handler);
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_app_register(ESP_SPP_APP_ID);
  // spp_task_init();
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(1100);
  if (local_mtu_ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x",
             local_mtu_ret);
  }
  return;
}
