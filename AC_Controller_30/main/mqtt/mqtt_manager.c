/* MQTT transport removed (Option B: Matter is to replace MQTT as the
 * northbound interface). This file previously hosted the esp-mqtt client,
 * broker/TLS connection, telemetry builders and publish path — all of
 * which are gone. What remains is the non-transport state and helpers
 * that the command/response logic in controller.c (and TimeManagement.c)
 * still depend on:
 *   - response_s            : command/response FSM state
 *   - Mqtt_is_connected     : transport-up flag, now permanently false
 *   - mac[]                 : device MAC (Manual config constant)
 *   - get_MAC_string()      : MAC formatter used in controller responses
 *   - ReadWriteNVS_MqttConfig() : NVS persistence for the MqttConfig command
 */
#include "main.h"
#include "mqtt_manager.h"
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "SafeFunctions/Jio_EA_Safe_Functions.h"

static const char *TAG = "mqtt_manager";

RESPONSE_s response_s;
bool Mqtt_is_connected = 0;

/* Device MAC. In the (active) Manual config the firmware used a fixed
 * value rather than the chip MAC; preserved here so get_MAC_string()
 * reports the same identity it did before the transport was removed. */
uint8_t mac[6] = {0xA0, 0x76, 0x4E, 0x53, 0x13, 0x84};

esp_err_t ReadWriteNVS_MqttConfig(uint8_t Type, MQTT_CONFIG_s *Mqtt_Config_t) {
  esp_err_t ret = ESP_OK;
  nvs_handle_t my_handle;
  ret = nvs_open("nvs", NVS_READWRITE, &my_handle);
  if (ret != ESP_OK) {
    ESP_LOGE("NVS", "Failed to open NVS handle ret-%s", esp_err_to_name(ret));
    return ret;
  }
  if (Type == 1 || Type == 2) { // write
    Mqtt_Config_t->Key = 0xAA;
    ret = nvs_set_blob(my_handle, "Mqtt_Config", Mqtt_Config_t,
                       sizeof(MQTT_CONFIG_s));
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to store string in NVS");
      nvs_close(my_handle);
      return ret;
    }
  } else { // read
    MQTT_CONFIG_s Mqtt_Config_tt = {0};
    size_t len = sizeof(MQTT_CONFIG_s);
    ret = nvs_get_blob(my_handle, "Mqtt_Config", &Mqtt_Config_tt, &len);
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to read string in NVS ret-%s",
               esp_err_to_name(ret));
      nvs_close(my_handle);
      return ret;
    }
    if (Mqtt_Config_tt.Key == 0xAA && len == sizeof(MQTT_CONFIG_s)) { // old
                                                                      // data
      memcpy_j(Mqtt_Config_t, sizeof(MQTT_CONFIG_s), (const void *)&Mqtt_Config_tt, len);
    } else {
      ESP_LOGE("NVS", "Failed Key mismatch ret-%s", esp_err_to_name(ret));
      nvs_close(my_handle);
      return ESP_ERR_INVALID_CRC;
    }
  }
  // Commit the changes to NVS
  ret = nvs_commit(my_handle);
  if (ret != ESP_OK) {
    ESP_LOGE("NVS", "Failed to commit data to NVS");
    nvs_close(my_handle);
    return ret;
  }
  ESP_LOGE("NVS",
          "%s Success in NVS under key '%s'",
          Type == 0 ? "Read" : "write", "Mqtt_Config");
  nvs_close(my_handle);
  ESP_LOGE("NVS", "%d",ret);
  return ret;
}

void get_MAC_string(char *dest,size_t dest_size)
{
	sprintf_j(dest, dest_size, "%02X:%02X:%02X:%02X:%02X:%02X",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

}
