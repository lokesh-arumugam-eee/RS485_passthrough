#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"
#include "lwip/inet.h"
typedef struct { uint32_t addr; } esp_ip4_addr_t;
#define ESP_IP4ADDR_INIT(a,b,c,d) {(uint32_t)(((d)<<24)|((c)<<16)|((b)<<8)|(a))}
typedef struct { esp_ip4_addr_t ip; esp_ip4_addr_t netmask; esp_ip4_addr_t gw; } esp_netif_ip_info_t;
typedef void* esp_netif_t;
typedef struct { esp_netif_ip_info_t ip_info; } ip_event_got_ip_t;
typedef enum {
    IP_EVENT_STA_GOT_IP=0,
    IP_EVENT_STA_LOST_IP,
    IP_EVENT_AP_STAIPASSIGNED,
    IP_EVENT_GOT_IP6,
    IP_EVENT_ETH_GOT_IP,
    IP_EVENT_MAX
} ip_event_t;
ESP_EVENT_DECLARE_BASE(IP_EVENT);
typedef union { esp_ip4_addr_t ip4; } esp_netif_ip_addr_u;
typedef struct { esp_netif_ip_addr_u u_addr; } esp_netif_ip_addr_t;
typedef struct { esp_netif_ip_addr_t ip; } esp_netif_dns_info_t;
typedef enum { ESP_NETIF_DNS_MAIN=0, ESP_NETIF_DNS_BACKUP, ESP_NETIF_DNS_FALLBACK, ESP_NETIF_DNS_MAX } esp_netif_dns_type_t;
static inline esp_err_t esp_netif_init(void){return ESP_OK;}
static inline esp_netif_t esp_netif_create_default_wifi_sta(void){return NULL;}
static inline esp_netif_t esp_netif_create_default_wifi_ap(void){return NULL;}
static inline void esp_netif_destroy_default_wifi(esp_netif_t netif){(void)netif;}
static inline esp_err_t esp_netif_get_ip_info(esp_netif_t n, esp_netif_ip_info_t *i){(void)n;if(i){i->ip.addr=0;}return ESP_OK;}
static inline esp_err_t esp_netif_set_ip_info(esp_netif_t n, const esp_netif_ip_info_t *i){(void)n;(void)i;return ESP_OK;}
static inline esp_netif_t esp_netif_get_handle_from_ifkey(const char*k){(void)k;return NULL;}
static inline esp_err_t esp_netif_dhcpc_stop(esp_netif_t n){(void)n;return ESP_OK;}
static inline esp_err_t esp_netif_set_dns_info(esp_netif_t n, esp_netif_dns_type_t t, esp_netif_dns_info_t *d){(void)n;(void)t;(void)d;return ESP_OK;}
#define IPSTR "%d.%d.%d.%d"
#define IP2STR(ip) (int)(((ip)->addr)&0xff),(int)((((ip)->addr)>>8)&0xff),(int)((((ip)->addr)>>16)&0xff),(int)((((ip)->addr)>>24)&0xff)
