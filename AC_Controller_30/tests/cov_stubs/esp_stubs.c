/* Runtime stub implementations for ESP-IDF non-static functions */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

/* esp_err_t */
typedef int32_t esp_err_t;
#define ESP_OK  0
#define ESP_FAIL -1

/* ------------------------------------------------------------------ */
/*  esp_netif                                                           */
/* ------------------------------------------------------------------ */
typedef struct { uint32_t addr; } esp_ip4_addr_t;
typedef struct { esp_ip4_addr_t ip; esp_ip4_addr_t netmask; esp_ip4_addr_t gw; } esp_netif_ip_info_t;
typedef void* esp_netif_t;

esp_netif_t esp_netif_create_default_wifi_sta(void)      { return NULL; }
esp_netif_t esp_netif_create_default_wifi_ap(void)       { return NULL; }
void esp_netif_destroy_default_wifi(esp_netif_t *esp_netif){ (void)esp_netif; }

/* ------------------------------------------------------------------ */
/*  esp_event loop (non-static additional helpers)                     */
/* ------------------------------------------------------------------ */
typedef const char* esp_event_base_t;
typedef void* esp_event_loop_handle_t;

/* Event base definitions (extern declared in esp_wifi.h / esp_netif.h) */
esp_event_base_t WIFI_EVENT = "WIFI_EVENT";
esp_event_base_t IP_EVENT   = "IP_EVENT";

/* ------------------------------------------------------------------ */
/*  FreeRTOS Queue stubs                                               */
/* ------------------------------------------------------------------ */
typedef uint32_t TickType_t;
typedef int      BaseType_t;
typedef void*    QueueHandle_t;

QueueHandle_t xQueueCreate(uint32_t uxQueueLength, uint32_t uxItemSize){
    (void)uxQueueLength; (void)uxItemSize; return (QueueHandle_t)malloc(1);
}
void vQueueDelete(QueueHandle_t xQueue){ free(xQueue); }
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait){
    (void)xQueue; (void)pvItemToQueue; (void)xTicksToWait; return 1;
}
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait){
    (void)xQueue; (void)pvBuffer; (void)xTicksToWait; return 0;
}
BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken){
    (void)xQueue; (void)pvItemToQueue; (void)pxHigherPriorityTaskWoken; return 1;
}

/* ------------------------------------------------------------------ */
/*  vTaskDelay / xTaskGetTickCount (non-static)                        */
/* ------------------------------------------------------------------ */
void    vTaskDelay(TickType_t xTicks){ (void)xTicks; }
/* Increment each call so retry/timeout loops eventually exit instead of
   spinning forever (e.g. uart_read_cb in modbus_rtu.c). */
TickType_t xTaskGetTickCount(void){ static TickType_t t = 0; return t++; }
void    vTaskDelete(void *h){ (void)h; }
BaseType_t xTaskCreate(void (*f)(void*), const char *n, uint32_t d, void *p, uint32_t prio, void **h){
    (void)f;(void)n;(void)d;(void)p;(void)prio;if(h)*h=NULL;return 1;
}
BaseType_t xTaskCreatePinnedToCore(void (*f)(void*), const char *n, uint32_t d, void *p, uint32_t prio, void **h, int core){
    (void)core; return xTaskCreate(f,n,d,p,prio,h);
}

/* ------------------------------------------------------------------ */
/*  Semaphore non-static helpers                                        */
/* ------------------------------------------------------------------ */
void* xSemaphoreCreateMutex(void){ return malloc(1); }
void* xSemaphoreCreateBinary(void){ return malloc(1); }
int   xSemaphoreTake(void *s, TickType_t t){ (void)s;(void)t; return 1; }
int   xSemaphoreGive(void *s){ (void)s; return 1; }
void  vSemaphoreDelete(void *s){ free(s); }

/* ------------------------------------------------------------------ */
/*  EventGroup non-static helpers                                       */
/* ------------------------------------------------------------------ */
typedef uint32_t EventBits_t;
void*      xEventGroupCreate(void){ return malloc(sizeof(uint32_t)); }
void       vEventGroupDelete(void *eg){ free(eg); }
EventBits_t xEventGroupSetBits(void *eg, EventBits_t bits){ (void)eg; return bits; }
EventBits_t xEventGroupClearBits(void *eg, EventBits_t bits){ (void)eg;(void)bits; return 0; }
EventBits_t xEventGroupWaitBits(void *eg, EventBits_t bw, int clr, int all, TickType_t t){ (void)eg;(void)clr;(void)all;(void)t; return bw; }
EventBits_t xEventGroupGetBits(void *eg){ (void)eg; return 0; }

/* ------------------------------------------------------------------ */
/*  esp_timer non-static                                                */
/* ------------------------------------------------------------------ */
typedef struct { void (*cb)(void*); void *arg; } esp_timer_impl_t;
typedef esp_timer_impl_t* esp_timer_handle_t;
typedef struct { void (*callback)(void*); void *arg; int dispatch_method; const char *name; bool skip_unhandled_events; } esp_timer_create_args_t;

esp_err_t esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *out){
    (void)a; if(out)*out=NULL; return ESP_OK;
}
esp_err_t esp_timer_start_once(esp_timer_handle_t t, uint64_t us){ (void)t;(void)us; return ESP_OK; }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t, uint64_t p){ (void)t;(void)p; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t t){ (void)t; return ESP_OK; }
esp_err_t esp_timer_delete(esp_timer_handle_t t){ (void)t; return ESP_OK; }
int64_t   esp_timer_get_time(void){ return 0; }
bool      esp_timer_is_active(esp_timer_handle_t t){ (void)t; return false; }

/* ------------------------------------------------------------------ */
/*  GPIO non-static                                                     */
/* ------------------------------------------------------------------ */
esp_err_t gpio_config(const void *cfg){ (void)cfg; return ESP_OK; }
esp_err_t gpio_set_direction(int pin, int mode){ (void)pin;(void)mode; return ESP_OK; }
esp_err_t gpio_set_level(int pin, uint32_t level){ (void)pin;(void)level; return ESP_OK; }
int       gpio_get_level(int pin){ (void)pin; return 0; }

/* ------------------------------------------------------------------ */
/*  LEDC non-static                                                     */
/* ------------------------------------------------------------------ */
esp_err_t ledc_timer_config(const void *cfg){ (void)cfg; return ESP_OK; }
esp_err_t ledc_channel_config(const void *cfg){ (void)cfg; return ESP_OK; }
esp_err_t ledc_set_duty(int m, int ch, uint32_t d){ (void)m;(void)ch;(void)d; return ESP_OK; }
esp_err_t ledc_update_duty(int m, int ch){ (void)m;(void)ch; return ESP_OK; }
uint32_t  ledc_get_duty(int m, int ch){ (void)m;(void)ch; return 0; }

/* ------------------------------------------------------------------ */
/*  UART non-static                                                     */
/* ------------------------------------------------------------------ */
esp_err_t uart_driver_install(int p, int rx, int tx, int q, QueueHandle_t *qu, int f){
    (void)p;(void)rx;(void)tx;(void)q;(void)qu;(void)f; return ESP_OK;
}
esp_err_t uart_param_config(int p, const void *cfg){ (void)p;(void)cfg; return ESP_OK; }
esp_err_t uart_set_pin(int p, int tx, int rx, int rts, int cts){ (void)p;(void)tx;(void)rx;(void)rts;(void)cts; return ESP_OK; }
int       uart_read_bytes(int p, void *buf, uint32_t len, TickType_t t){ (void)p;(void)buf;(void)len;(void)t; return 0; }
int       uart_write_bytes(int p, const void *src, size_t sz){ (void)p;(void)src; return (int)sz; }
esp_err_t uart_flush(int p){ (void)p; return ESP_OK; }
esp_err_t uart_flush_input(int p){ (void)p; return ESP_OK; }
esp_err_t uart_wait_tx_done(int p, TickType_t t){ (void)p;(void)t; return ESP_OK; }
esp_err_t uart_get_buffered_data_len(int p, size_t *sz){ (void)p;if(sz)*sz=0; return ESP_OK; }
esp_err_t uart_set_baudrate(int p, uint32_t b){ (void)p;(void)b; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  NVS non-static                                                      */
/* ------------------------------------------------------------------ */
extern esp_err_t _stub_nvs_open_ret;
extern esp_err_t _stub_nvs_commit_ret;
extern esp_err_t _stub_nvs_set_blob_ret;
extern esp_err_t _stub_nvs_get_blob_ret;
extern int       _stub_nvs_corrupt_key;
esp_err_t nvs_open(const char *n, int m, void **h){
    (void)n;(void)m;
    if (_stub_nvs_open_ret != ESP_OK) return _stub_nvs_open_ret;
    if (h) *h = (void*)0x1; /* non-NULL handle so `if(my_handle)` true branch runs */
    return ESP_OK;
}
esp_err_t nvs_close(void *h){ (void)h; return ESP_OK; }
esp_err_t nvs_commit(void *h){ (void)h; return _stub_nvs_commit_ret; }
/* nvs_set_blob / nvs_get_blob are defined later with a real in-memory store. */
esp_err_t nvs_set_str(void *h, const char *k, const char *v){ (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_get_str(void *h, const char *k, char *v, size_t *l){ (void)h;(void)k;if(v&&l&&*l>0)v[0]='\0'; return ESP_OK; }
esp_err_t nvs_set_u8(void *h, const char *k, uint8_t v){ (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_get_u8(void *h, const char *k, uint8_t *v){ (void)h;(void)k;if(v)*v=0; return ESP_OK; }
esp_err_t nvs_set_i32(void *h, const char *k, int32_t v){ (void)h;(void)k;(void)v; return ESP_OK; }
esp_err_t nvs_get_i32(void *h, const char *k, int32_t *v){ (void)h;(void)k;if(v)*v=0; return ESP_OK; }
esp_err_t nvs_erase_key(void *h, const char *k){ (void)h;(void)k; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  MAC                                                                 */
/* ------------------------------------------------------------------ */
esp_err_t esp_read_mac(uint8_t *mac, int type){
    (void)type;
    if(mac){ mac[0]=0xAA;mac[1]=0xBB;mac[2]=0xCC;mac[3]=0xDD;mac[4]=0xEE;mac[5]=0xFF; }
    return ESP_OK;
}
esp_err_t esp_base_mac_addr_get(uint8_t *mac){ return esp_read_mac(mac,0); }

/* ------------------------------------------------------------------ */
/*  BLE stubs                                                           */
/* ------------------------------------------------------------------ */
esp_err_t esp_bt_controller_init(void *cfg){ (void)cfg; return ESP_OK; }
esp_err_t esp_bt_controller_enable(uint8_t m){ (void)m; return ESP_OK; }
esp_err_t esp_bt_controller_disable(void){ return ESP_OK; }
esp_err_t esp_bt_controller_deinit(void){ return ESP_OK; }
esp_err_t esp_bt_mem_release(uint8_t m){ (void)m; return ESP_OK; }
esp_err_t esp_bt_controller_mem_release(uint8_t m){ (void)m; return ESP_OK; }
esp_err_t esp_bluedroid_init(void){ return ESP_OK; }
esp_err_t esp_bluedroid_enable(void){ return ESP_OK; }
esp_err_t esp_bluedroid_disable(void){ return ESP_OK; }
esp_err_t esp_bluedroid_deinit(void){ return ESP_OK; }
esp_err_t esp_ble_gap_register_callback(void *cb){ (void)cb; return ESP_OK; }
esp_err_t esp_ble_gap_config_adv_data(void *d){ (void)d; return ESP_OK; }
esp_err_t esp_ble_gap_set_device_name(const char *n){ (void)n; return ESP_OK; }
esp_err_t esp_ble_gap_start_advertising(void *p){ (void)p; return ESP_OK; }
esp_err_t esp_ble_gap_stop_advertising(void){ return ESP_OK; }
esp_err_t esp_ble_gatts_register_callback(void *cb){ (void)cb; return ESP_OK; }
esp_err_t esp_ble_gatts_app_register(uint16_t id){ (void)id; return ESP_OK; }
esp_err_t esp_ble_gatts_create_attr_tab(const void *db, uint8_t gi, uint16_t n, uint8_t sid){ (void)db;(void)gi;(void)n;(void)sid; return ESP_OK; }
esp_err_t esp_ble_gatts_start_service(uint16_t h){ (void)h; return ESP_OK; }
esp_err_t esp_ble_gatts_send_indicate(uint8_t gi, uint16_t ci, uint16_t ah, uint16_t vl, uint8_t *v, bool nc){ (void)gi;(void)ci;(void)ah;(void)vl;(void)v;(void)nc; return ESP_OK; }
esp_err_t esp_ble_gatts_send_response(uint8_t gi, uint16_t ci, uint32_t tid, int st, void *rsp){ (void)gi;(void)ci;(void)tid;(void)st;(void)rsp; return ESP_OK; }
esp_err_t esp_ble_gatt_set_local_mtu(uint16_t mtu){ (void)mtu; return ESP_OK; }
const uint8_t *esp_bt_dev_get_address(void){ static uint8_t a[6]={0}; return a; }
esp_err_t esp_bt_dev_set_device_name(const char *n){ (void)n; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  MQTT stub (non-static)                                              */
/* ------------------------------------------------------------------ */
int        _stub_mqtt_init_null        = 0;
esp_err_t  _stub_mqtt_start_ret        = 0; /* ESP_OK */
esp_err_t  _stub_mqtt_register_evt_ret = 0;
int        _stub_mqtt_publish_ret      = 0;
void* esp_mqtt_client_init(const void *cfg){ (void)cfg; return _stub_mqtt_init_null ? NULL : (void*)(uintptr_t)0xCAFEBABE; }
esp_err_t esp_mqtt_client_start(void *c){ (void)c; return _stub_mqtt_start_ret; }
esp_err_t esp_mqtt_client_stop(void *c){ (void)c; return ESP_OK; }
esp_err_t esp_mqtt_client_destroy(void *c){ (void)c; return ESP_OK; }
int esp_mqtt_client_publish(void *c, const char *t, const char *d, int l, int q, int r){ (void)c;(void)t;(void)d;(void)l;(void)q;(void)r; return _stub_mqtt_publish_ret; }
int esp_mqtt_client_subscribe(void *c, const char *t, int q){ (void)c;(void)t;(void)q; return 0; }
/* esp_wifi runtime stubs (so we can drive failure paths in firmware) */
typedef int wifi_mode_t;
esp_err_t _stub_wifi_set_config_ret_storage = 0; /* dummy if not yet defined */
extern esp_err_t _stub_wifi_set_config_ret;
extern esp_err_t _stub_wifi_start_ret;
extern esp_err_t _stub_wifi_stop_ret;
esp_err_t esp_wifi_init(const void *c){ (void)c; return ESP_OK; }
esp_err_t esp_wifi_start(void){ return _stub_wifi_start_ret; }
esp_err_t esp_wifi_stop(void){ return _stub_wifi_stop_ret; }
esp_err_t esp_wifi_connect(void){ return ESP_OK; }
esp_err_t esp_wifi_disconnect(void){ return ESP_OK; }
esp_err_t esp_wifi_set_mode(wifi_mode_t m){ (void)m; return ESP_OK; }
esp_err_t esp_wifi_set_config(int i, void *c){ (void)i; (void)c; return _stub_wifi_set_config_ret; }
esp_err_t _stub_wifi_get_ap_info_ret = 0; /* ESP_OK by default */
esp_err_t esp_wifi_sta_get_ap_info(void *r){ (void)r; return _stub_wifi_get_ap_info_ret; }

/* Mqtt handler dispatch (declared in mqtt_client.h stub) */
typedef void (*_stub_mqtt_evt_handler_t)(void *handler_args, const char *base, int32_t event_id, void *event_data);
_stub_mqtt_evt_handler_t _stub_mqtt_handler = NULL;
void *_stub_mqtt_handler_arg = NULL;

/* BLE GAP/GATTS callback storage so coverage tests can dispatch BLE events
   to firmware's static event handlers. */
typedef void (*_stub_gap_cb_t)(int event, void *param);
typedef void (*_stub_gatts_cb_t)(int event, int gatts_if, void *param);
_stub_gap_cb_t   _stub_gap_cb   = NULL;
_stub_gatts_cb_t _stub_gatts_cb = NULL;
esp_err_t esp_mqtt_client_register_event(void *c, int e, void *h, void *a){ (void)c;(void)e; _stub_mqtt_handler=(_stub_mqtt_evt_handler_t)h; _stub_mqtt_handler_arg=a; return _stub_mqtt_register_evt_ret; }
esp_err_t esp_mqtt_set_config(void *c, const void *cfg){ (void)c;(void)cfg; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  HTTP client stub (non-static)                                       */
/* ------------------------------------------------------------------ */
void* esp_http_client_init(const void *cfg){ (void)cfg; return (void*)(uintptr_t)0xDEADBEEF; }
/* Stub-controlled HTTP behaviour. Tests can poke these to drive download_task
   and get_ota_manifest through their data-processing branches. */
int      _stub_http_read_count = 0;       /* 0 = no data, N = return N bytes once then 0 */
int64_t  _stub_http_content_length = 0;   /* what fetch_headers returns */
const char *_stub_http_read_data = NULL;  /* if non-NULL, copy this on the first read */
esp_err_t _stub_http_perform_ret = ESP_OK;
esp_err_t _stub_http_open_ret    = ESP_OK;
int       _stub_http_status_code = 200;

/* Application-owned OTA state. In firmware these are defined in
   ota/chunked_ota.c (formerly ota/http.c, now removed). chunked_ota.c is
   not part of the coverage build, so the harness defines them here to
   satisfy references from main.c, modbus_rtu.c, configuration.c, etc. */
#include "http.h"
ota_params_t httpserver_params;
int otataskstarted = 0;
int g_download_progress = 0;
esp_err_t esp_http_client_perform(void *c){ (void)c; return _stub_http_perform_ret; }
esp_err_t esp_http_client_cleanup(void *c){ (void)c; return ESP_OK; }
esp_err_t esp_http_client_open(void *c, int l){ (void)c;(void)l; return _stub_http_open_ret; }
esp_err_t esp_http_client_close(void *c){ (void)c; return ESP_OK; }
int esp_http_client_get_status_code(void *c){ (void)c; return _stub_http_status_code; }
int64_t esp_http_client_get_content_length(void *c){ (void)c; return 0; }
int esp_http_client_read(void *c, char *buf, int len){
    (void)c;
    if (_stub_http_read_count > 0) {
        int n = (_stub_http_read_count < len) ? _stub_http_read_count : len;
        if (_stub_http_read_data && buf) {
            memcpy(buf, _stub_http_read_data, n);
            _stub_http_read_data += n;
        } else if (buf) {
            memset(buf, 0xAB, n);
        }
        _stub_http_read_count -= n;
        return n;
    }
    return 0;
}
int esp_http_client_write(void *c, const char *buf, int len){ (void)c;(void)buf; return len; }
esp_err_t esp_http_client_set_header(void *c, const char *k, const char *v){ (void)c;(void)k;(void)v; return ESP_OK; }
esp_err_t esp_http_client_set_url(void *c, const char *u){ (void)c;(void)u; return ESP_OK; }
esp_err_t esp_http_client_set_method(void *c, int m){ (void)c;(void)m; return ESP_OK; }
esp_err_t esp_http_client_set_post_field(void *c, const char *d, int l){ (void)c;(void)d;(void)l; return ESP_OK; }
int64_t   esp_http_client_fetch_headers(void *c){ (void)c; return _stub_http_content_length; }
bool      esp_http_client_is_complete_data_received(void *c){ (void)c; return true; }

/* ------------------------------------------------------------------ */
/*  OTA non-static                                                      */
/* ------------------------------------------------------------------ */
typedef struct { uint8_t type; uint8_t subtype; uint32_t address; uint32_t size; char label[16]; bool encrypted; } esp_partition_t;
typedef void* esp_ota_handle_t;
static esp_partition_t _running_part = {0,0x10,0x10000,0x100000,"app0",false};
static esp_partition_t _next_part    = {0,0x11,0x110000,0x100000,"ota_0",false};
int _stub_ota_running_null = 0; /* if 1, esp_ota_get_running_partition returns NULL */
int _stub_ota_next_null    = 0; /* if 1, esp_ota_get_next_update_partition returns NULL */
esp_err_t _stub_ota_begin_ret      = ESP_OK;
esp_err_t _stub_ota_write_ret      = ESP_OK;
esp_err_t _stub_ota_end_ret        = ESP_OK;
esp_err_t _stub_ota_set_boot_ret   = ESP_OK;
const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t *s){ (void)s; return _stub_ota_next_null ? NULL : &_next_part; }
const esp_partition_t* esp_ota_get_running_partition(void){ return _stub_ota_running_null ? NULL : &_running_part; }
const esp_partition_t* esp_ota_get_boot_partition(void){ return &_running_part; }
/* Partition iterator — when _stub_partition_iter_count > 0, yield that many
   iterations of _running_part / _next_part alternating. Use void* in the
   signature here to avoid pulling in esp_ota_ops.h's typedefs. */
int _stub_partition_iter_count = 0;
static int _stub_partition_iter_pos  = 0;
void* esp_partition_find(int type, int subtype, const char *label){
    (void)type;(void)subtype;(void)label;
    if (_stub_partition_iter_count <= 0) return NULL;
    _stub_partition_iter_pos = 0;
    return (void*)(uintptr_t)1;
}
const esp_partition_t* esp_partition_get(void *iterator){
    (void)iterator;
    return (_stub_partition_iter_pos & 1) ? &_next_part : &_running_part;
}
void* esp_partition_next(void *iterator){
    _stub_partition_iter_pos++;
    if (_stub_partition_iter_pos >= _stub_partition_iter_count) return NULL;
    return iterator;
}
void esp_partition_iterator_release(void *iterator){ (void)iterator; }
esp_err_t esp_ota_begin(const esp_partition_t *p, size_t sz, esp_ota_handle_t *h){ (void)p;(void)sz;if(h)*h=(esp_ota_handle_t)0x1234; return _stub_ota_begin_ret; }
esp_err_t esp_ota_write(esp_ota_handle_t h, const void *d, size_t sz){ (void)h;(void)d;(void)sz; return _stub_ota_write_ret; }
esp_err_t esp_ota_end(esp_ota_handle_t h){ (void)h; return _stub_ota_end_ret; }
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *p){ (void)p; return _stub_ota_set_boot_ret; }
esp_err_t esp_ota_abort(esp_ota_handle_t h){ (void)h; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  SPIFFS / VFS                                                        */
/* ------------------------------------------------------------------ */
esp_err_t _stub_spiffs_register_ret = ESP_OK;
esp_err_t _stub_spiffs_info_ret     = ESP_OK;
size_t    _stub_spiffs_info_used_override = 0;

/* Generic stub error knobs — tests can toggle these to drive firmware error paths. */
esp_err_t _stub_wifi_set_config_ret  = ESP_OK;
esp_err_t _stub_wifi_start_ret       = ESP_OK;
esp_err_t _stub_wifi_stop_ret        = ESP_OK;
esp_err_t _stub_nvs_open_ret         = ESP_OK;
esp_err_t _stub_nvs_commit_ret       = ESP_OK;
esp_err_t _stub_nvs_set_blob_ret     = ESP_OK;
esp_err_t _stub_nvs_get_blob_ret     = ESP_OK;  /* if non-OK, nvs_get_blob returns this */
int       _stub_nvs_corrupt_key      = 0;       /* if 1, get_blob returns OK but with garbage Key field */
size_t    _stub_nvs_get_len_override = 0;       /* if nonzero, *length forced to this value (drives len-mismatch firmware paths) */
esp_err_t _stub_event_register_ret   = ESP_OK;
int       _stub_socket_ret           = 0;     /* if -1, socket() returns -1 */
int       _stub_connect_ret          = 0;     /* if -1, connect() returns -1 */
esp_err_t esp_vfs_spiffs_register(const void *cfg){ (void)cfg; return _stub_spiffs_register_ret; }
esp_err_t esp_vfs_spiffs_unregister(const char *label){ (void)label; return ESP_OK; }
esp_err_t esp_spiffs_info(const char *label, size_t *total, size_t *used){
    (void)label;
    if (total) *total = 1048576;
    if (used)  *used  = _stub_spiffs_info_used_override ? _stub_spiffs_info_used_override : 0;
    return _stub_spiffs_info_ret;
}
bool      esp_spiffs_mounted(const char *label){ (void)label; return true; }
esp_err_t _stub_spiffs_check_ret = 0; /* ESP_OK by default; tests poke to drive failure */
esp_err_t esp_spiffs_check(const char *label){ (void)label; return _stub_spiffs_check_ret; }
esp_err_t esp_spiffs_format(const char *label){ (void)label; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  I2C non-static                                                      */
/* ------------------------------------------------------------------ */
esp_err_t i2c_param_config(int p, const void *cfg){ (void)p;(void)cfg; return ESP_OK; }
esp_err_t i2c_driver_install(int p, int m, size_t rx, size_t tx, int f){ (void)p;(void)m;(void)rx;(void)tx;(void)f; return ESP_OK; }
void*     i2c_cmd_link_create(void){ return malloc(1); }
void      i2c_cmd_link_delete(void *h){ free(h); }
esp_err_t i2c_master_start(void *h){ (void)h; return ESP_OK; }
esp_err_t i2c_master_stop(void *h){ (void)h; return ESP_OK; }
esp_err_t i2c_master_write_byte(void *h, uint8_t d, bool a){ (void)h;(void)d;(void)a; return ESP_OK; }
esp_err_t i2c_master_write(void *h, const uint8_t *d, size_t l, bool a){ (void)h;(void)d;(void)l;(void)a; return ESP_OK; }
esp_err_t i2c_master_read_byte(void *h, uint8_t *d, int a){ (void)h;if(d)*d=0;(void)a; return ESP_OK; }
esp_err_t i2c_master_read(void *h, uint8_t *d, size_t l, int a){ (void)h;if(d&&l)memset(d,0,l);(void)a; return ESP_OK; }
esp_err_t i2c_master_cmd_begin(int p, void *h, TickType_t t){ (void)p;(void)h;(void)t; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  ping stubs                                                          */
/* ------------------------------------------------------------------ */
esp_err_t esp_ping_new_session(const void *cfg, const void *cb, void **hdl){ (void)cfg;(void)cb;if(hdl)*hdl=NULL; return ESP_OK; }
esp_err_t esp_ping_start(void *h){ (void)h; return ESP_OK; }
esp_err_t esp_ping_stop(void *h){ (void)h; return ESP_OK; }
esp_err_t esp_ping_delete_session(void *h){ (void)h; return ESP_OK; }
esp_err_t esp_ping_get_profile(void *h, int p, void *d, uint32_t s){ (void)h;(void)p;(void)d;(void)s; return ESP_OK; }

/* ------------------------------------------------------------------ */
/*  esp_system non-static (for TUs that don't include esp_system.h)   */
/* ------------------------------------------------------------------ */
void esp_restart(void){}

/* ------------------------------------------------------------------ */
/*  NVS in-memory blob store (so set/get round-trips correctly)        */
/* ------------------------------------------------------------------ */
#define _NVS_MAX_BLOBS 16
#define _NVS_MAX_KEY    32
#define _NVS_MAX_BLOB   512
typedef struct {
    char key[_NVS_MAX_KEY];
    uint8_t data[_NVS_MAX_BLOB];
    size_t  len;
    int     used;
} _nvs_blob_t;
static _nvs_blob_t _nvs_blobs[_NVS_MAX_BLOBS];

esp_err_t nvs_set_blob(void *handle, const char *key, const void *value, size_t length) {
    (void)handle;
    if (_stub_nvs_set_blob_ret != ESP_OK) return _stub_nvs_set_blob_ret;
    if (!key || !value || length > _NVS_MAX_BLOB) return ESP_FAIL;
    /* Update existing key */
    for (int i = 0; i < _NVS_MAX_BLOBS; i++) {
        if (_nvs_blobs[i].used && strncmp(_nvs_blobs[i].key, key, _NVS_MAX_KEY) == 0) {
            memcpy(_nvs_blobs[i].data, value, length);
            _nvs_blobs[i].len = length;
            return ESP_OK;
        }
    }
    /* Insert new */
    for (int i = 0; i < _NVS_MAX_BLOBS; i++) {
        if (!_nvs_blobs[i].used) {
            strncpy(_nvs_blobs[i].key, key, _NVS_MAX_KEY - 1);
            _nvs_blobs[i].key[_NVS_MAX_KEY - 1] = '\0';
            memcpy(_nvs_blobs[i].data, value, length);
            _nvs_blobs[i].len = length;
            _nvs_blobs[i].used = 1;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}
esp_err_t nvs_get_blob(void *handle, const char *key, void *out_value, size_t *length) {
    (void)handle;
    if (_stub_nvs_get_blob_ret != ESP_OK) return _stub_nvs_get_blob_ret;
    if (!key || !length) return 0x1102;  /* ESP_ERR_NVS_NOT_FOUND */
    for (int i = 0; i < _NVS_MAX_BLOBS; i++) {
        if (_nvs_blobs[i].used && strncmp(_nvs_blobs[i].key, key, _NVS_MAX_KEY) == 0) {
            size_t to_copy = (_nvs_blobs[i].len < *length) ? _nvs_blobs[i].len : *length;
            if (out_value) {
                if (_stub_nvs_corrupt_key) {
                    /* Drive the firmware's "Key mismatch" else branch */
                    memset(out_value, 0x00, to_copy);
                } else {
                    memcpy(out_value, _nvs_blobs[i].data, to_copy);
                }
            }
            /* If the test forces a length, override; lets us drive the
               "Key==0xAA && len != sizeof" else branch without disturbing
               the stored data's Key byte. */
            *length = _stub_nvs_get_len_override ? _stub_nvs_get_len_override
                                                  : _nvs_blobs[i].len;
            return ESP_OK;
        }
    }
    if (length) *length = 0;
    return 0x1102; /* ESP_ERR_NVS_NOT_FOUND */
}

/* ------------------------------------------------------------------ */
/*  esp_event with shared dispatch table across all TUs                */
/* ------------------------------------------------------------------ */
#include "esp_event.h"
#define _STUB_EVT_MAX 16
typedef struct {
    esp_event_base_t base;
    int32_t          id;
    esp_event_handler_t handler;
    void            *arg;
    int              used;
} _stub_evt_entry_t;
static _stub_evt_entry_t _stub_evt_table[_STUB_EVT_MAX];

static esp_err_t _stub_evt_register(esp_event_base_t b, int32_t id, esp_event_handler_t h, void *a) {
    for (int i = 0; i < _STUB_EVT_MAX; i++) {
        if (!_stub_evt_table[i].used) {
            _stub_evt_table[i].base = b;
            _stub_evt_table[i].id = id;
            _stub_evt_table[i].handler = h;
            _stub_evt_table[i].arg = a;
            _stub_evt_table[i].used = 1;
            return ESP_OK;
        }
    }
    return ESP_OK;
}
esp_err_t esp_event_loop_create_default(void){return ESP_OK;}
esp_err_t esp_event_loop_create(const esp_event_loop_args_t *a, esp_event_loop_handle_t *l){(void)a;(void)l;return ESP_OK;}
esp_err_t esp_event_loop_delete_default(void){return ESP_OK;}
esp_err_t esp_event_handler_register(esp_event_base_t b, int32_t id, esp_event_handler_t h, void *a){
    return _stub_evt_register(b, id, h, a);
}
esp_err_t esp_event_handler_register_with(esp_event_loop_handle_t l, esp_event_base_t b, int32_t id, esp_event_handler_t h, void *a){
    (void)l; return _stub_evt_register(b, id, h, a);
}
esp_err_t esp_event_handler_instance_register(esp_event_base_t b, int32_t id, esp_event_handler_t h, void *a, esp_event_handler_instance_t *inst){
    if (inst) *inst = NULL;
    return _stub_evt_register(b, id, h, a);
}
esp_err_t esp_event_handler_unregister(esp_event_base_t b, int32_t id, esp_event_handler_t h){
    (void)b; (void)id;
    for (int i = 0; i < _STUB_EVT_MAX; i++) {
        if (_stub_evt_table[i].used && _stub_evt_table[i].handler == h) {
            _stub_evt_table[i].used = 0;
        }
    }
    return ESP_OK;
}
esp_err_t esp_event_post(esp_event_base_t b, int32_t id, void *d, size_t ds, uint32_t tw){
    (void)ds; (void)tw;
    for (int i = 0; i < _STUB_EVT_MAX; i++) {
        if (!_stub_evt_table[i].used) continue;
        if (_stub_evt_table[i].base == b &&
            (_stub_evt_table[i].id == ESP_EVENT_ANY_ID || _stub_evt_table[i].id == id)) {
            _stub_evt_table[i].handler(_stub_evt_table[i].arg, b, id, d);
        }
    }
    return ESP_OK;
}
esp_err_t esp_event_post_to(esp_event_loop_handle_t l, esp_event_base_t b, int32_t id, void *d, size_t ds, uint32_t tw){
    (void)l; return esp_event_post(b, id, d, ds, tw);
}

/* ------------------------------------------------------------------ */
/*  Firmware functions declared in headers but never defined           */
/* ------------------------------------------------------------------ */
#include <time.h>
__attribute__((weak)) void set_current_time(time_t t){ (void)t; }
__attribute__((weak)) int  get_cmd_type(unsigned char b){ (void)b; return 0; }
__attribute__((weak)) int  ReadFromFile(const char *path, unsigned char *buf, int len){ (void)path;(void)buf;(void)len; return 0; }
__attribute__((weak)) int  WriteIntoFile(const char *path, const unsigned char *buf, int len){ (void)path;(void)buf;(void)len; return 0; }
__attribute__((weak)) void LIR_task_init(void){}
