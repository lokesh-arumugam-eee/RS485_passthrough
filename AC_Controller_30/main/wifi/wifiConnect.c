#include "esp_err.h"
#include "main.h"
#include <stdint.h>

#include "xmodem.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "wifiConnect.h"
#include <string.h>
#include "lwip/ip4_addr.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_ping.h"
#include "lwip/inet.h"
#include "configuration.h"   /* ip_config / total_ip_configs (was pulled transitively via mqtt_manager.h) */
#include "modbus_tcp.h"      /* tcp_app_init() — Modbus TCP gateway + HTTP config server */
/* The examples use WiFi configuration that you can set via project
   configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
/* BUG-FIX #5: previously baked dev-network credentials ("Info_2.4G" /
   "Pair$3848") into every shipping binary — recoverable from any field
   unit by trivial string-extract. Defaults are now empty; the device
   refuses to associate without provisioned credentials.
   TESTING ONLY: placeholder values below let local builds boot past the
   Wi-Fi wait without stalling. Do not ship a binary with these set. */
#define EXAMPLE_ESP_WIFI_SSID "TestSSID"
#define EXAMPLE_ESP_WIFI_PASS "TestPassword123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 25
WIFICONFIG_S wifiConfig = {
    .ssid = EXAMPLE_ESP_WIFI_SSID,
    .password = EXAMPLE_ESP_WIFI_PASS,
    .RtryC = EXAMPLE_ESP_MAXIMUM_RETRY,
};
#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;
bool wifi_is_connected = false;
esp_netif_ip_info_t ip_info;
bool static_ip_enabled=false;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    s_retry_num = 0;
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < wifiConfig.RtryC) {
      wifi_is_connected = false;
      acm_log("wifi down\n");
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
       s_retry_num = 0;
       wifi_init_sta();
     }
    ESP_LOGI(TAG, "AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    acm_log("wifi up ip=" IPSTR "\n", IP2STR(&event->ip_info.ip));
    memcpy_j(&ip_info,sizeof(ip_info),&event->ip_info, sizeof(ip_info));
    s_retry_num = 0;
    wifi_is_connected = true;

    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    /* Bring up the Modbus TCP gateway + HTTP config server once we
       actually have an IP. Guarded so reconnect events (which re-fire
       IP_EVENT_STA_GOT_IP) don't call tcp_app_init() again and try to
       bind/listen on sockets that are already open. */
    static bool tcp_services_started = false;
    if (!tcp_services_started) {
      if (ip_config != NULL && ip_config[0].St_ip_En_dis == 1) {
        tcp_app_init();
      } else {
        ESP_LOGI("MAIN", "TCP is Disabled");
      }
      tcp_services_started = true;
    }
  }
}
// Function to change Wi-Fi SSID and Password
esp_err_t change_wifi_credentials(const char *new_ssid,
                                  const char *new_password) {
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = {0},
          .password = {0},
          .bssid_set = false, // Ensure we don't lock to a specific BSSID
      }};
// Copy the new SSID and password to the Wi-Fi config

  // if (ReadWriteNVS_wifiConfig(0, &wifiConfig) == ESP_OK) {
  //   ;
  // }
  
  int8_t return_value=0;
  return_value = strcpy_j((char *)wifi_config.sta.ssid, 
                               sizeof(wifi_config.sta.ssid), 
                               (const char *)wifiConfig.ssid, 
                               strlen((const char *)wifiConfig.ssid)); 
    printf("return value of ssid: %d\n", return_value);

  return_value = strcpy_j((char *)wifi_config.sta.password, 
                               sizeof(wifi_config.sta.password), 
                               (const char *)wifiConfig.password, 
                               strlen((const char *)wifiConfig.password)); 
    printf("return value of password: %d\n", return_value);


  // Set the new Wi-Fi configuration
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());
  // ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA,
  // &wifi_config) );
  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  if (ret != ESP_OK) {
    ESP_LOGE("Wi-Fi", "Failed to set New Wi-Fi config: %s",
             esp_err_to_name(ret));
    return ret;
  }
  s_retry_num = 0;
  // Reconnect to the new network
  ret = esp_wifi_start();
  if (ret != ESP_OK) {
    ESP_LOGE("Wi-Fi", "Failed to connect: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t ReadWriteNVS_wifiConfig(uint8_t Type, WIFICONFIG_S *wifiConfig_t) {
  esp_err_t ret = ESP_OK;
#if 0 // Enable only if NVS init not done else where
		 ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK(ret);
#endif
  nvs_handle_t my_handle;
  ret = nvs_open("nvs", NVS_READWRITE, &my_handle);
  if (ret != ESP_OK) {
    ESP_LOGE("NVS", "Failed to open NVS handle");
    return ret;
  }
  if (Type == 1 || Type == 2) { // write
    wifiConfig_t->Key = 0xAA;
    ret = nvs_set_blob(my_handle, "wifiConfig", wifiConfig_t,
                       sizeof(WIFICONFIG_S));
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to store string in NVS");
      nvs_close(my_handle);
      return ret;
    }
  } else { // read
    WIFICONFIG_S wifiConfig_tt = {0};
    size_t len = sizeof(WIFICONFIG_S);
    ret = nvs_get_blob(my_handle, "wifiConfig", &wifiConfig_tt, &len);
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to read string in NVS ret-%s",
               esp_err_to_name(ret));
      nvs_close(my_handle);
      return ret;
    }
    if (wifiConfig_tt.Key == 0xAA && len == sizeof(WIFICONFIG_S)) { // old data
      memcpy_j(wifiConfig_t, sizeof(WIFICONFIG_S), (const void *)&wifiConfig_tt, len);
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

  ESP_LOGE("NVS:","%s Success SSID:'%s', PASS:'%s' in NVS under key '%s'",
          Type == 0 ? "Read" : "write", wifiConfig_t->ssid,
          wifiConfig_t->password, "wifiConfig");
  ESP_LOGE("NVS","Closing handle");        
  if(my_handle)        
   nvs_close(my_handle);

  // if (ret == ESP_OK && Type == 2)
  //   ret = change_wifi_credentials((const char *)wifiConfig_t->ssid,
  //                                 (const char *)wifiConfig_t->password);

  ESP_LOGE("NVS","after setting handle");  
  return ret;
}
esp_err_t ReadWriteNVS(uint8_t Type, uint8_t *data, char *key, int length) {
  esp_err_t ret = ESP_OK;
#if 0 // Enable only if NVS init not done else where
		 ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK(ret);
#endif
  nvs_handle_t my_handle;
  ret = nvs_open("nvs", NVS_READWRITE, &my_handle);
  if (ret != ESP_OK) {
    ESP_LOGE("NVS", "Failed to open NVS handle");
    return ret;
  }
  if (Type == 1 || Type == 2) { // write

    ret = nvs_set_blob(my_handle, key, data, length);
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to store string in NVS");
      nvs_close(my_handle);
      return ret;
    }
  } else { // read
    ret = nvs_get_blob(my_handle, key, data, (size_t *)&length);
    if (ret != ESP_OK) {
      ESP_LOGE("NVS", "Failed to read string in NVS ret-%s",
               esp_err_to_name(ret));
      nvs_close(my_handle);
      return ret;
    }
  }
  // Commit the changes to NVS
  ret = nvs_commit(my_handle);
  if (ret != ESP_OK) {
    ESP_LOGE("NVS", "Failed to commit data to NVS");
    nvs_close(my_handle);
    return ret;
  }
  return ret;
}

void wifi_init_sta(void)
{
    static bool WifiInitDone = false;
    static wifi_config_t wifi_config;

    if (!WifiInitDone) {
        s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());
        /* Controller_task (started by Controller_init(), which now runs
           before this) may create the default event loop concurrently on
           its own task. Tolerate "already created" instead of aborting. */
        esp_err_t loop_ret = esp_event_loop_create_default();
        if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(loop_ret);
        }
        esp_netif_create_default_wifi_sta();

        esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

        if (ip_config!=NULL && ip_config[0].St_ip_En_dis == 1 &&
                            ip_config[0].st_ip_info.ip.addr != 0 &&
                            ip_config[0].st_ip_info.netmask.addr != 0 &&
                            ip_config[0].st_ip_info.gw.addr != 0) {
          ip_info.ip.addr      = ip_config[0].st_ip_info.ip.addr;
          ip_info.netmask.addr = ip_config[0].st_ip_info.netmask.addr;
          ip_info.gw.addr      = ip_config[0].st_ip_info.gw.addr;
        
        ESP_LOGI("NVS", "Loaded Static IP from Flash: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI("NVS", "\nLoaded Static netmask from Flash: " IPSTR, IP2STR(&ip_info.netmask));
        ESP_LOGI("NVS", "\nLoaded Static GW from Flash: " IPSTR, IP2STR(&ip_info.gw));
        // IP4_ADDR(&ip_info.ip, 10, 189, 31, 100); // Static IP
        // IP4_ADDR(&ip_info.gw, 100, 115, 96, 151);        // Gateway
        // IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Netmask
         esp_netif_dhcpc_stop(sta_netif); // Must stop DHCP client first
         esp_netif_set_ip_info(sta_netif, &ip_info);
         static_ip_enabled = true;

         esp_netif_dns_info_t dns_info;
        IP4_ADDR(&dns_info.ip.u_addr.ip4, 8, 8, 8, 8); // Google Public DNS
        esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info);
        } else {
            ESP_LOGI("NVS", "Static IP disabled in config, using DHCP");
            static_ip_enabled = false;
        }

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Register handlers only once
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

        WifiInitDone = true;
    } else {
        // If Wi-Fi was previously started, stop before reconfiguring
        esp_wifi_stop();

    }
      uint8_t return_value=0;
      return_value = strcpy_j((char *)wifi_config.sta.ssid, 
                               sizeof(wifi_config.sta.ssid), 
                               (const char *)wifiConfig.ssid, 
                               strlen((const char *)wifiConfig.ssid)); 
        printf("return value of ssid: %d\n", return_value);
          return_value = strcpy_j((char *)wifi_config.sta.password, 
                               sizeof(wifi_config.sta.password), 
                               (const char *)wifiConfig.password, 
                               strlen((const char *)wifiConfig.password)); 
    printf("return value of password: %d\n", return_value);
    printf("STA.SSID: %s, STA.Password: %s\n", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_max_tx_power(20);

    ESP_LOGI(TAG, "Wi-Fi started, waiting for connection...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to SSID:%s Password:%s", wifi_config.sta.ssid, wifi_config.sta.password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", wifi_config.sta.ssid);
    } else {
        ESP_LOGE(TAG, "Unexpected Wi-Fi event bits: 0x%lx", bits);
    }
}

/*
static void ping_results(ping_target_id_t msgType, esp_ping_found *pf)
{
    if (msgType == PING_TARGET_RES_FINISH) {
        if (pf->received_count > 0) {
            ESP_LOGI("PING_TEST", "Internet available");
        } else {
            ESP_LOGW("PING_TEST", "No internet access");
        }
    }
}

void check_internet(void)
{
    ip_addr_t target_addr;
    inet_pton(AF_INET, "8.8.8.8", &target_addr.u_addr.ip4);
    target_addr.type = IPADDR_TYPE_V4;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.target_addr = target_addr;   // Google DNS

    esp_ping_callbacks_t cbs = {
        .on_ping_end = ping_results,
    };

    esp_ping_handle_t ping;
    esp_ping_new_session(&ping_config, &cbs, &ping);
    esp_ping_start(ping);
}
*/
/*
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret ==
ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); ret =
nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}*/
