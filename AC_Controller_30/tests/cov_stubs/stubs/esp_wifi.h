#pragma once
#include "esp_err.h"
#include "esp_event.h"
#include <stdint.h>
#include <stdbool.h>
typedef enum { WIFI_MODE_NULL=0, WIFI_MODE_STA, WIFI_MODE_AP, WIFI_MODE_APSTA } wifi_mode_t;
typedef enum { WIFI_AUTH_OPEN=0, WIFI_AUTH_WEP, WIFI_AUTH_WPA_PSK, WIFI_AUTH_WPA2_PSK, WIFI_AUTH_WPA_WPA2_PSK, WIFI_AUTH_WPA2_ENTERPRISE, WIFI_AUTH_WPA3_PSK, WIFI_AUTH_WPA2_WPA3_PSK, WIFI_AUTH_WAPI_PSK, WIFI_AUTH_MAX } wifi_auth_mode_t;
typedef enum {
    WIFI_EVENT_WIFI_READY=0,
    WIFI_EVENT_SCAN_DONE,
    WIFI_EVENT_STA_START,
    WIFI_EVENT_STA_STOP,
    WIFI_EVENT_STA_CONNECTED,
    WIFI_EVENT_STA_DISCONNECTED,
    WIFI_EVENT_STA_AUTHMODE_CHANGE,
    WIFI_EVENT_STA_WPS_ER_SUCCESS,
    WIFI_EVENT_STA_WPS_ER_FAILED,
    WIFI_EVENT_STA_WPS_ER_TIMEOUT,
    WIFI_EVENT_AP_START,
    WIFI_EVENT_AP_STOP,
    WIFI_EVENT_AP_STACONNECTED,
    WIFI_EVENT_AP_STADISCONNECTED,
    WIFI_EVENT_MAX
} wifi_event_t;
typedef enum { WIFI_STORAGE_FLASH=0, WIFI_STORAGE_RAM } wifi_storage_t;
typedef struct { uint8_t ssid[32]; uint8_t password[64]; bool bssid_set; uint8_t bssid[6]; wifi_auth_mode_t threshold_authmode; bool sae_pwe_h2e; } wifi_sta_config_t;
typedef union { wifi_sta_config_t sta; } wifi_config_t;
typedef struct { int8_t rssi; uint8_t ssid[33]; uint8_t bssid[6]; } wifi_ap_record_t;
ESP_EVENT_DECLARE_BASE(WIFI_EVENT);
/* Forward decls — definitions in esp_stubs.c so coverage tests can poke
   _stub_wifi_*_ret knobs to drive firmware error paths. */
extern esp_err_t _stub_wifi_set_config_ret;
extern esp_err_t _stub_wifi_start_ret;
extern esp_err_t _stub_wifi_stop_ret;
esp_err_t esp_wifi_init(const void*c);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_stop(void);
esp_err_t esp_wifi_connect(void);
esp_err_t esp_wifi_disconnect(void);
esp_err_t esp_wifi_set_mode(wifi_mode_t m);
esp_err_t esp_wifi_set_config(int i, wifi_config_t *c);
static inline esp_err_t esp_wifi_get_config(int i,wifi_config_t*c){(void)i;(void)c;return ESP_OK;}
extern esp_err_t _stub_wifi_get_ap_info_ret;
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t*r);
static inline esp_err_t esp_wifi_set_storage(wifi_storage_t storage){(void)storage;return ESP_OK;}
static inline esp_err_t esp_wifi_set_max_tx_power(int8_t power){(void)power;return ESP_OK;}
#define WIFI_IF_STA 0
#define WIFI_IF_AP  1
typedef struct {} wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() {}
