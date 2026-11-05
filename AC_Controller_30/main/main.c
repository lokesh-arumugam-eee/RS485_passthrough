#include "main.h"
#include "xmodem/xmodem.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "mqtt_manager.h"
#include "status_led.h"
#include "multi_heap.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"
#include "wifiConnect.h"
#include "ota/ota.h"
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

#include "ble_spp/ble_spp_server_demo.h"
#include "controller/controller.h"
#include "TimeManagement.h"
#include "mqtt_manager.h"
#include "uart/uart_app.h"
#include "configuration/configuration.h"
#include "http.h"
#include "http_server.h"
#include "chunked_ota.h"
#include "log_capture.h"
#include "sensors.h"
#include "modbus_rtu.h"
#define ACM_LOG_TAG "ACM_main"

uart_event_t event;
void cstm_app_main(void);

	uint8_t mac_ble[6];
	uint8_t mac_Wifi[6];
	uint8_t frequency=38;
	uint8_t irlenlim=0;

char version[16];
char release_date[16];

/* OTA completion report destination — change these (or set from config/Modbus
   register) to redirect where the firmware-upgrade result is POSTed. */
char     ota_report_ip[64]   = "192.168.8.193";
uint16_t ota_report_port     = 1180;
char     ota_report_path[64] = "/ota/report";
/* When set non-zero, app_main exits after one while-loop iteration. Defined
   here so the host coverage harness can call app_main once to exercise its
   initialization and main-loop branches. Not used (and harmless) on target. */
int g_app_main_test_break = 0;
	
void ota_diag(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
   // ESP_LOGI("OTA_DIAG", "Running partition: label=%s, type=0x%x, subtype=0x%x, addr=0x%x, size=%d", running->label, running->type, running->subtype, running->address, running->size);
 ESP_LOGI("OTA_DIAG", "Running from: %s type=0x%x,subtype=0x%x,addr=0x%x,size=%"PRIu32,running->label, (unsigned int)running->type, (unsigned int)running->subtype,
                (unsigned int)running->address, running->size);
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (next) {
		ESP_LOGI("OTA_DIAG", "Next OTA partition: %s at 0x%x size=%"PRIu32,next->label,(unsigned int) next->address,next->size);
        //ESP_LOGI("OTA_DIAG", "Next OTA partition: %s at 0x%x size=%d", next->label, next->address, next->size);
    } else {
        ESP_LOGE("OTA_DIAG", "esp_ota_get_next_update_partition(NULL) returned NULL");
    }

    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it) {
        const esp_partition_t *p = esp_partition_get(it);
        ESP_LOGI("OTA_DIAG", "Partition: label=%s subtype=0x%x addr=0x%x size=%"PRIu32,p->label,(unsigned int) p->subtype, (unsigned int)p->address, p->size);
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
}
void endislog()
{
 if(excutelog==1)
 {
	 excutelog=0;
	 ReadWriteNVS(1, (uint8_t *)&log_endis, "LOG_EN", 1);
	 if(log_endis==1)
	 {
	 	  
		  // Enable only errors
	esp_log_level_set("*", ESP_LOG_ERROR);

	// Enable info and above
	esp_log_level_set("*", ESP_LOG_INFO);
	 }
	 else {
	esp_log_level_set("*", ESP_LOG_NONE);
     }
 }		
}

void Mount_SPIFFS_Flash(void) {
    ESP_LOGI(ACM_LOG_TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    // 1. Register Filesystem
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        const char* err_msg = (ret == ESP_FAIL) ? "Failed to mount/format" :
                              (ret == ESP_ERR_NOT_FOUND) ? "Partition not found" : "Init failed";
        ESP_LOGE(ACM_LOG_TAG, "%s (%s)", err_msg, esp_err_to_name(ret));
        return;
    }

    // 2. Get Partition Info
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(ACM_LOG_TAG, "Failed to get info. Formatting...");
        esp_spiffs_format(conf.partition_label);
        return;
    }

    ESP_LOGI(ACM_LOG_TAG, "Partition size: total: %d, used: %d", total, used);

    // 3. Consistency Check (Guard Clause Style)
    if (used <= total) {
        return; // Everything is fine, exit early
    }

    ESP_LOGW(ACM_LOG_TAG, "Used > Total. Running SPIFFS_check().");
    ret = esp_spiffs_check(conf.partition_label);
    
    if (ret != ESP_OK) {
        ESP_LOGE(ACM_LOG_TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(ACM_LOG_TAG, "SPIFFS_check() successful");
    }
}

void app_main() {
	ESP_LOGI("RESET", "Reason: %d", esp_reset_reason());
    sprintf_j(version, sizeof(version), "%03d:%03d:%03d",FW_VERHIGH,FW_VERLOW,FW_VERPATCH);
    sprintf_j(release_date, sizeof(release_date), "%02d:%02d:%04d",REL_DA,REL_MO,REL_YR);
	//ota_diag();
	const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
	esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);

	ESP_LOGI(ACM_LOG_TAG, "FW VERSION: %s,REL_DATE:%s",version,release_date);
     // LED_init();
    status_led_init();
    // Enable only errors
    esp_log_level_set("*", ESP_LOG_ERROR);

    // Enable info and above
    esp_log_level_set("*", ESP_LOG_INFO);

	esp_read_mac(mac_ble, ESP_MAC_BT);
	esp_read_mac(mac_Wifi, ESP_MAC_WIFI_STA);

  char device_name[63];
  uint16_t len=0;
  
  len = sprintf_j(device_name, sizeof(device_name), "{\"BLE_MAC\":\"%02X:%02X:%02X:%02X:%02X:%02X\",",mac_ble[0],mac_ble[1],mac_ble[2],mac_ble[3],mac_ble[4],mac_ble[5]);
  sprintf_j(device_name+len, sizeof(device_name)-len, "\"WIFI_MAC\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",mac_Wifi[0],mac_Wifi[1],mac_Wifi[2],mac_Wifi[3],mac_Wifi[4],mac_Wifi[5]);
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
        // Load config (from NVS or static)

    if (ReadWriteNVS_wifiConfig(0, &wifiConfig) != ESP_OK) {
        /* No provisioned config in NVS yet — keep the compiled-in
           wifiConfig defaults (set at declaration) instead of zeroing
           them out, so a fresh device still has something to try. */
        printf("Failed to read Wi-Fi config from NVS, using compiled defaults\n");
    }

  Mount_SPIFFS_Flash();
  //delete_config_file(config_path);

 if (!file_exists(config_path))
 {
      ESP_LOGW("STORAGE", "Config file missing, creating default...");
      save_config_json();
}

 // Usage in app_main:
 if (file_exists(config_path)) {
     ESP_LOGI("STORAGE", "Config file found!");

     char* config_raw = read_config_to_buffer(config_path);
     ESP_LOGI("JSON", "%s",config_raw);
	  if (config_raw != NULL) {
		 //PARSE: Hydrate your dynamic structs (the function we just built)
		  init_system_settings(config_raw);
		  parse_mbr_settings(config_raw);
		  parse_profile_settings(config_raw);
		 //CLEANUP: Free the temporary text buffer
		  free(config_raw);
		  Check_config();
		  debug_print_configs();
	  }
 }

 /* If the previous boot crashed mid-capture, /spiffs/debug.log is still
    on flash. Pull it into RAM, delete the on-flash copy, and spawn a
    task that uploads it as <mac>_recovered.log once Wi-Fi is up. No-op
    if nothing was left behind. */
 log_capture_recover_on_boot();

  ble_gap_gatt_spp_init();

  ESP_LOGI("MAIN","%s",device_name);
  //ESP_LOGI("MAIN","DISABLING ALL LOGS");
 // if(log_endis==0)
 //esp_log_level_set("*", ESP_LOG_NONE);

  /* Must run before wifi_init_sta(): Controller_init() starts
     Controller_task, which registers the AC_CNTRL_EVENT_BASE handler
     that parse_WifiConfigCmd() (BLE wifiConfig command) depends on.
     wifi_init_sta() blocks for a long time trying to connect/retry, so
     if it ran first, any wifiConfig command sent over BLE during that
     window would be posted to the event loop with nobody subscribed
     yet and get silently dropped. */
  Controller_init();

  /* Must run before wifi_init_sta(): modbus_app_init() creates
     xModbus_Mutex, brings up the RS485 UART, and sets up the RTU master
     context. The TCP gateway / pass-through tasks (started once Wi-Fi
     gets an IP, see tcp_app_init() in wifiConnect.c) take xModbus_Mutex
     on first real request — with it still NULL that hits FreeRTOS's
     configASSERT in xQueueSemaphoreTake and aborts the device. */
  modbus_app_init();

  /* Must run before anything that touches lwIP (e.g. SNTP in
     TimeMngmt_init): esp_netif_init() inside wifi_init_sta() is what
     spins up the lwIP TCP/IP task. Without it, the first lwIP call
     asserts on an uninitialized mbox ("tcpip_callback ... Invalid
     mbox") and reboots the device. */
  wifi_init_sta();

  TimeMngmt_init();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  //i2c_master_init();
  //Read_sensor(AIR_QUALITY_SENSOR);
  uart_init();

  int length=0;
  int count =0;
  int ota_count =0;
  i2c_mutex = xSemaphoreCreateMutex();

    if (i2c_mutex == NULL) {
        printf("Failed to create I2C mutex\n");
    }
//   xTaskCreate(Read_sensor_task, "Read_sensor_task", 1024 * 2, NULL, 5, NULL);
//   memset_j(server_url, sizeof(server_url), 0, sizeof(server_url));
  while (1) {

	  vTaskDelay(pdMS_TO_TICKS(500));
	  //endislog();
    //---------------------------------------
      // Wait for UART events (RX FIFO full, timeout, etc.)
      if (xQueueReceive(uart_queue, &event, pdMS_TO_TICKS(1000))) {
          if (event.type == UART_DATA) {
              length = uart_read_bytes(UART_PORT, (void *)uart_rx_buffer, event.size, 0);
              if (length > 0) {
                  esp_event_post(AC_CNTRL_EVENT_BASE, EVENT_ID_UART_DATA_RX, (void *)uart_rx_buffer, length + 1, portMAX_DELAY);
              }
          }
      }
      //----------------------------------------------------------------------------------

    //   if(count++>1)
    //   {
    //     Read_sensor(AIR_QUALITY_SENSOR);
    //     count=0;
    //     // ESP_LOGI("MAIN", "Reading sensor data\n");
    //     // Read_sensor(PRECISION_LIGHT);
    //     // vTaskDelay(pdMS_TO_TICKS(10));
    //     // Read_sensor(TEMP_HUMIDITY);
    //     // vTaskDelay(pdMS_TO_TICKS(10));
    //     // Read_sensor(HIGH_ACC_HUMIDITY_TEMP);
    //     // vTaskDelay(pdMS_TO_TICKS(10));
    //     // Read_sensor(THREE_AXIS_MAGNETOMETER);
    //     // vTaskDelay(pdMS_TO_TICKS(10)); 
    //   }
        /* log_params.trigger==1 means "capture debug logs and upload".
           Settings (duration, URL path) come from log_params; IP/port
           still come from httpserver_params. log_capture_task clears
           the trigger and otataskstarted when done. */
     if(log_params.trigger==1 && ota_log_started == 0)
            {
                ota_log_started = 1;
                ESP_LOGI("OTA", "Starting log capture task");
                log_capture_start();
                ota_count = 0;
            }
      //--------------------------------------------------------------------------------------
      if((httpserver_params.is_ota==1||httpserver_params.is_ota==2||httpserver_params.is_ota==100) && otataskstarted== 0)
      {

        // time_t now;
        // char strftime_buf[64];
        // struct tm timeinfo;
        // time(&now);
        // localtime_r(&now, &timeinfo);
        // strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        // ESP_LOGI("TIME_CHECK", "The current date/time is: %s", strftime_buf);
        ota_count++;
        printf("OTA Check Count: %d\n", ota_count);

        /* Once per boot, after Wi-Fi has had a chance to come up, deliver
           any "OTA succeeded, please tell the server" record left in NVS
           by the previous (pre-reboot) firmware. */
        if(httpserver_params.is_ota==100 && otataskstarted== 0)
            {
                static bool pending_report_tried = false;
                if (!pending_report_tried) {
                    pending_report_tried = true;
                    chunked_ota_send_pending_report();
                }
                ota_count=0;

            }

        /* Heartbeat: tell the server "alive, about to poll for OTA".
           Report destination = same host:port as the firmware server. */
        else if(ota_count==1){
            chunked_ota_post_report(httpserver_params.url, httpserver_params.port,
                                    ota_report_path,
                                    httpserver_params.filepath,
                                    OTA_IDLE, OTA_FAILURE_DEFAULT);
        }
        if(ota_count>29)
        {
            ota_count=0;
            /* OTA source (url/port/scheme/filename) is read from
               httpserver_params inside chunked_ota_run; cfg only carries
               the report destination + retry tuning. */
            chunked_ota_config_t cfg = {0};
            httpserver_params.ota_status = 0;
            httpserver_params.ota_percentage = 0;
            httpserver_params.ota_failure_reason = 0;
            /* Report destination tracks the firmware server (same host:port). */
            snprintf(cfg.report_ip,   sizeof(cfg.report_ip),   "%s", httpserver_params.url);
            snprintf(cfg.report_path, sizeof(cfg.report_path), "%s", ota_report_path);
            cfg.report_port         = httpserver_params.port;
            cfg.requested_pkt_size  = 2048;
            cfg.max_session_retries = 3;
            cfg.max_packet_retries  = 3;
            chunked_ota_start_task(&cfg);
        }
      }
      /* Test-mode escape hatch: never fires on target (flag stays 0). */
      if (g_app_main_test_break) break;
  }
}
