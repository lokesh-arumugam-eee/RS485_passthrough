#pragma once
#include "esp_err.h"
#include <stdint.h>

typedef enum {
    ESP_MAC_WIFI_STA = 0,
    ESP_MAC_WIFI_SOFTAP,
    ESP_MAC_BT,
    ESP_MAC_ETH,
    ESP_MAC_IEEE802154,
    ESP_MAC_LOCAL,
    ESP_MAC_UNIVERSAL,
} esp_mac_type_t;

static inline esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type){(void)type;if(mac){mac[0]=0xAA;mac[1]=0xBB;mac[2]=0xCC;mac[3]=0xDD;mac[4]=0xEE;mac[5]=0xFF;}return ESP_OK;}
static inline esp_err_t esp_base_mac_addr_get(uint8_t *mac){if(mac){mac[0]=0xAA;mac[1]=0xBB;mac[2]=0xCC;mac[3]=0xDD;mac[4]=0xEE;mac[5]=0xFF;}return ESP_OK;}
static inline esp_err_t esp_base_mac_addr_set(const uint8_t *mac){(void)mac;return ESP_OK;}
