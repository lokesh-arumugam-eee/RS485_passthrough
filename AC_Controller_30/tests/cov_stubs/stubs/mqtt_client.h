#pragma once
#include "esp_err.h"
#include "mqtt_supported_features.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void* esp_mqtt_client_handle_t;

typedef enum {
    MQTT_EVENT_ERROR=0,
    MQTT_EVENT_CONNECTED,
    MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_SUBSCRIBED,
    MQTT_EVENT_UNSUBSCRIBED,
    MQTT_EVENT_PUBLISHED,
    MQTT_EVENT_DATA,
    MQTT_EVENT_BEFORE_CONNECT,
    MQTT_EVENT_DELETED,
    MQTT_EVENT_ANY=-1
} esp_mqtt_event_id_t;

typedef enum { MQTT_TRANSPORT_OVER_TCP=0, MQTT_TRANSPORT_OVER_SSL, MQTT_TRANSPORT_OVER_WS, MQTT_TRANSPORT_OVER_WSS } esp_mqtt_transport_t;

typedef enum {
    MQTT_ERROR_TYPE_NONE=0,
    MQTT_ERROR_TYPE_TCP_TRANSPORT,
    MQTT_ERROR_TYPE_CONNECTION_REFUSED
} esp_mqtt_error_type_t;

typedef struct {
    esp_mqtt_error_type_t error_type;
    int esp_tls_last_esp_err;
    int esp_tls_stack_err;
    int esp_transport_sock_errno;
} esp_mqtt_error_codes_t;

typedef struct {
    int event_id;
    esp_mqtt_client_handle_t client;
    void *user_context;
    char *data;
    int data_len;
    int total_data_len;
    int current_data_offset;
    char *topic;
    int topic_len;
    int msg_id;
    int session_present;
    esp_mqtt_error_codes_t *error_handle;
    bool retain;
    int qos;
    bool dup;
} esp_mqtt_event_t;

typedef esp_mqtt_event_t* esp_mqtt_event_handle_t;
typedef void (*esp_mqtt_event_callback_t)(esp_mqtt_event_handle_t event);

/* ESP-IDF v5 nested MQTT config struct */
typedef struct {
    struct {
        struct {
            const char *uri;
            const char *host;
            uint32_t port;
            const char *path;
            esp_mqtt_transport_t transport;
        } address;
        struct {
            const char *certificate;
            size_t certificate_len;
            bool skip_cert_common_name_check;
            bool use_global_ca_store;
            bool (*crt_bundle_attach)(void *);
        } verification;
    } broker;
    struct {
        const char *client_id;
        const char *username;
        struct {
            const char *password;
            const char *certificate;
            size_t certificate_len;
            const char *key;
            size_t key_len;
            const char *key_password;
            int key_password_len;
        } authentication;
    } credentials;
    struct {
        int keepalive;
        bool disable_keepalive;
        bool disable_clean_session;
        const char *lwt_topic;
        const char *lwt_msg;
        int lwt_qos;
        int lwt_retain;
        int lwt_msg_len;
        int protocol_ver;
    } session;
    struct {
        int task_prio;
        uint32_t task_stack;
        int buffer_size;
        int out_buffer_size;
        int reconnect_timeout_ms;
        int network_timeout_ms;
        int refresh_connection_after_ms;
        bool disable_auto_reconnect;
        void *user_context;
        int message_retransmit_timeout;
        const char **alpn_protos;
    } network;
} esp_mqtt_client_config_t;

/* Forward decls — implementations in esp_stubs.c so coverage tests can
   poke knobs to drive failure paths in firmware. */
extern int        _stub_mqtt_init_null;       /* if 1, esp_mqtt_client_init returns NULL */
extern esp_err_t  _stub_mqtt_start_ret;       /* esp_mqtt_client_start return value */
extern esp_err_t  _stub_mqtt_register_evt_ret;/* esp_mqtt_client_register_event return value */
extern int        _stub_mqtt_publish_ret;     /* esp_mqtt_client_publish return value (-1 = fail) */
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client);
static inline esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client){(void)client;return ESP_OK;}
static inline esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client){(void)client;return ESP_OK;}
static inline esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client){(void)client;return ESP_OK;}
static inline esp_err_t esp_mqtt_client_disconnect(esp_mqtt_client_handle_t client){(void)client;return ESP_OK;}
int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain);
static inline int esp_mqtt_client_enqueue(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain, bool store){(void)client;(void)topic;(void)data;(void)len;(void)qos;(void)retain;(void)store;return 0;}
static inline int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos){(void)client;(void)topic;(void)qos;return 0;}
static inline int esp_mqtt_client_unsubscribe(esp_mqtt_client_handle_t client, const char *topic){(void)client;(void)topic;return 0;}
/* Remember the registered handler so coverage tests can dispatch fake events
   to exercise the static mqtt_event_handler in firmware code. */
typedef void (*_stub_mqtt_evt_handler_t)(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
extern _stub_mqtt_evt_handler_t _stub_mqtt_handler;
extern void *_stub_mqtt_handler_arg;
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, esp_mqtt_event_id_t event, void *event_handler, void *event_handler_arg);
static inline esp_err_t esp_mqtt_set_config(esp_mqtt_client_handle_t client, const esp_mqtt_client_config_t *config){(void)client;(void)config;return ESP_OK;}
