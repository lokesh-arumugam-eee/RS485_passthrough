#pragma once
#include "esp_netif.h"
#define ip4_addr_t esp_ip4_addr_t
#define IP4ADDR_STRLEN_MAX 16
static inline uint32_t ip4addr_addr(const char* addr_str){(void)addr_str;return 0;}
static inline char* ip4addr_ntoa(const ip4_addr_t *addr){(void)addr;return "";}
static inline char* ip4addr_ntoa_r(const ip4_addr_t *addr, char *buf, int buflen){(void)addr;if(buf&&buflen>0)buf[0]='\0';return buf;}
static inline int ip4addr_aton(const char *cp, ip4_addr_t *addr){(void)cp;if(addr)addr->addr=0;return 0;}
#define IP4_ADDR(ipaddr,a,b,c,d) do{(ipaddr)->addr=((uint32_t)(d)<<24)|((uint32_t)(c)<<16)|((uint32_t)(b)<<8)|(uint32_t)(a);}while(0)
#define ip4_addr_get_u32(addr) ((addr)->addr)
#define PP_HTONL(x) __builtin_bswap32(x)
#define PP_NTOHS(x) __builtin_bswap16(x)
