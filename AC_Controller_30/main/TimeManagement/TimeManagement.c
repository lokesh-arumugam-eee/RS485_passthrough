#include "main.h"
#include <stdint.h>
#include "TimeManagement.h"
#include "sys/time.h"
#include "wifiConnect.h"

#include "controller.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "mqtt_manager.h"
#include "xmodem.h"

#define GPIO_WAKEUP_PIN GPIO_NUM_0 // You can change to any available GPIO

#if (MODULE_TYPE == TARGET_ESP32_C3)
#define LED_GPIO GPIO_NUM_18// Define the output GPIO
#elif (MODULE_TYPE == TARGET_ESP32_S3)
#define LED_GPIO GPIO_NUM_10// Define the output GPIO
#elif (MODULE_TYPE == TARGET_ESP32_S3_WAVESHARE)
#define LED_GPIO GPIO_NUM_10// Define the output GPIO
#else
#define LED_GPIO GPIO_NUM_1 // Define the output GPIO
#endif

static const char *TAG = "TIME_MNGMT";
bool time_is_synced = false;
time_t now;
struct tm timeinfo;

int32_t hr_min_sec,pre_hr_min_sec;
//holiday_t holidays[10] = {{1, 26,25}, {8, 15,25}, {10, 2,25}, {12, 25,25}};
//const int holiday_count = sizeof(holidays) / sizeof(holidays[0]);
void init_sntp();
void set_timezone();
void wait_for_time_sync();
//bool is_today_holiday(struct tm *t);
//bool just_started_schedule(struct tm *t);
//bool just_ended_schedule(struct tm *t);
void time_sync_notification_cb(struct timeval *tv) {
  ESP_LOGI(TAG, "Time synchronized via SNTP (callback)");
  time_is_synced = true;
}
void init_sntp() {
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
   esp_sntp_setservername(1, "time.cloudflare.com");
    esp_sntp_setservername(2, "time.google.com");
  sntp_set_time_sync_notification_cb(time_sync_notification_cb);
  esp_sntp_init();
}
void set_timezone() {
  setenv("TZ", "IST-5:30", 1); // Replace with your region
  tzset();
}
void wait_for_time_sync() {
  int retry = 0;
  const int retry_count = 10;

  while (!time_is_synced && ++retry < retry_count) {
    ESP_LOGI(TAG, "Waiting for SNTP sync... (%d/%d)", retry, retry_count);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (time_is_synced) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
  } else {
    ESP_LOGI(TAG, "Time sync failed after retries");
  }
}

void TimeMngmt_init(void) {
	  init_sntp();
  wait_for_time_sync();
  set_timezone();
	
	
  //xTaskCreate(TimeMngmt_task, "Time_Task", 1024 * 2, NULL, 8, NULL);
}
void LedMngmt_task(void *arg) {
  while (1) {
     LED_Logic();
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void LED_init(void) {
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
  ESP_LOGI(TAG, "LED TASK");
  xTaskCreate(LedMngmt_task, "Led_blinking_Task", 1024*2, NULL, 8, NULL);
}

void LED_Logic(void) {
  static led_mode_t Led_State = LED_CONTINOUS_ON;
  static uint8_t Delay_count = 8, LED_Status_ONOFF = 0;

  if(IR_Transmit_blink)
  {
	if(Led_State != LED_LONGON_BLINK)
			{
		      Led_State = LED_LONGON_BLINK;
		      ESP_LOGI(TAG, "LED_LONGON_BLINK");
		      Delay_count=0;
		     }
  }
 else if(CMD_Recieved)
	{
	if(Led_State != LED_BLINK_THRICE)  
		{
	      Led_State = LED_BLINK_THRICE;
	      ESP_LOGI(TAG, "LED_BLINK_THRICE");
	      Delay_count=0;
	     }	
	}
 else if (time_is_synced== false && wifi_is_connected== true) {
	if(Led_State != LED_BLINK_TWICE)
	{
      Led_State = LED_BLINK_TWICE;
      ESP_LOGI(TAG, "LED_BLINK_TWICE");
     }
	}
  else if (wifi_is_connected== true)
  {
	if(Led_State != LED_SLOW_BLINKING)
	{
      Led_State = LED_SLOW_BLINKING;
     // ESP_LOGI(TAG, "LED_SLOW_BLINKING");
    }
  }
  else 
  {
	if (Led_State != LED_CONTINOUS_ON) {
      Led_State = LED_CONTINOUS_ON;
      //ESP_LOGI(TAG, "LED_CONTINOUS_ON");
  }  
  }

  switch (Led_State) {
  case LED_CONTINOUS_ON:
    if (LED_Status_ONOFF == 0) {
      LED_Status_ONOFF = 1;
      gpio_set_level(LED_GPIO, 1);
    }
    break;
  case LED_SLOW_BLINKING:
   if(Delay_count>0)
      Delay_count--;
    if (Delay_count == 0) {
      if (LED_Status_ONOFF == 1) {
        LED_Status_ONOFF = 0;
        gpio_set_level(LED_GPIO, 0);
      } else if (LED_Status_ONOFF == 0) {
        LED_Status_ONOFF = 1;
        gpio_set_level(LED_GPIO, 1);
      }
      Delay_count = 8;
    }
    break;
  case LED_FAST_BLINKING:
	  if(Delay_count>0)
	  Delay_count--;
    if (Delay_count == 0) {
      if (LED_Status_ONOFF == 1) {
        LED_Status_ONOFF = 0;
        gpio_set_level(LED_GPIO, 0);
      } else if (LED_Status_ONOFF == 0) {
        LED_Status_ONOFF = 1;
        gpio_set_level(LED_GPIO, 1);
      }
      Delay_count = 1;
    }
    break;
    case LED_BLINK_TWICE:
    if(Delay_count>0)
	  Delay_count--;
     if (Delay_count == 0)
     {
		  LED_Status_ONOFF = 1;
		  gpio_set_level(LED_GPIO, 1);
		 vTaskDelay(100 / portTICK_PERIOD_MS); 
		  gpio_set_level(LED_GPIO, 0);
		  vTaskDelay(100 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 1);
		   vTaskDelay(100 / portTICK_PERIOD_MS);
		    gpio_set_level(LED_GPIO, 0);
		    Delay_count = 8;
		     LED_Status_ONOFF = 0;
	 }
	break;
	case LED_BLINK_THRICE:
	 if(Delay_count>0)
	  Delay_count--;
     if (Delay_count == 0)
     {
		  LED_Status_ONOFF = 1;
		  gpio_set_level(LED_GPIO, 1);
		 vTaskDelay(100 / portTICK_PERIOD_MS); 
		  gpio_set_level(LED_GPIO, 0);
		  vTaskDelay(100 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 1);
		   vTaskDelay(100 / portTICK_PERIOD_MS);
		    gpio_set_level(LED_GPIO, 0);
		    vTaskDelay(100 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 1);
		   vTaskDelay(100 / portTICK_PERIOD_MS);
		    gpio_set_level(LED_GPIO, 0);
		    Delay_count = 8;
		     LED_Status_ONOFF = 0;
		     CMD_Recieved=0;
	 }
	break;
  case LED_LONGON_BLINK:
		  gpio_set_level(LED_GPIO, 0);
		  vTaskDelay(100 / portTICK_PERIOD_MS); 
		  gpio_set_level(LED_GPIO, 1);
		  vTaskDelay(500 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 0);
		   vTaskDelay(100 / portTICK_PERIOD_MS);
		    gpio_set_level(LED_GPIO, 1);
		    vTaskDelay(500 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 0);
		    vTaskDelay(100 / portTICK_PERIOD_MS);
		   gpio_set_level(LED_GPIO, 1);
		   vTaskDelay(500 / portTICK_PERIOD_MS);
		    gpio_set_level(LED_GPIO, 0);
		    Delay_count = 8;
		     LED_Status_ONOFF = 0;
         IR_Transmit_blink=0;
  break;

  }
} 
