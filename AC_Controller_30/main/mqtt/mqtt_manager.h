
#ifndef MQTT_L
#define MQTT_L
/* MQTT transport removed (Matter is to replace it). Only the
   non-transport state/helpers consumed by controller.c and
   TimeManagement.c remain — see mqtt_manager.c. */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "configuration.h"
/*------------------------------------------------*/

#define BROKER_URL_MAX_LEN 126
#define MQTT_PSW_MAX_LEN 64

typedef struct {
  uint8_t BROKER_URL[BROKER_URL_MAX_LEN];
  uint8_t Device_Psw[MQTT_PSW_MAX_LEN];
  uint8_t Key;
} MQTT_CONFIG_s;

typedef struct {
  uint32_t tid;
  uint8_t cmd;
  uint8_t res;
  uint8_t rev_cmd;
} RESPONSE_s;
extern RESPONSE_s response_s;

/* Private includes ----------------------------------------------------------*/
extern esp_err_t ReadWriteNVS_MqttConfig(uint8_t Type, MQTT_CONFIG_s *Mqtt_Config_t);
extern void get_MAC_string(char *dest,size_t dest_size);
extern bool Mqtt_is_connected;
#endif /*MQTT_L */
