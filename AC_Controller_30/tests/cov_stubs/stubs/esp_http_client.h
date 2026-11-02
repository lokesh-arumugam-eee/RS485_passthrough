#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef void* esp_http_client_handle_t;
typedef enum { HTTP_METHOD_GET=0, HTTP_METHOD_POST, HTTP_METHOD_PUT, HTTP_METHOD_DELETE, HTTP_METHOD_HEAD, HTTP_METHOD_PATCH } esp_http_client_method_t;
typedef enum { HTTP_AUTH_TYPE_NONE=0, HTTP_AUTH_TYPE_BASIC, HTTP_AUTH_TYPE_DIGEST } esp_http_client_auth_type_t;
typedef enum { HTTP_TRANSPORT_UNKNOWN=0, HTTP_TRANSPORT_OVER_TCP, HTTP_TRANSPORT_OVER_SSL, HTTP_TRANSPORT_OVER_WS, HTTP_TRANSPORT_OVER_WSS } esp_http_client_transport_t;

typedef struct {
    int event_id;
    esp_http_client_handle_t client;
    char *data;
    int data_len;
    int status_code;
    uint64_t content_length;
} esp_http_client_event_t;

typedef esp_err_t (*http_event_handle_cb)(esp_http_client_event_t *evt);

typedef struct {
    const char *url;
    const char *host;
    int port;
    const char *username;
    const char *password;
    esp_http_client_auth_type_t auth_type;
    const char *path;
    const char *query;
    const char *cert_pem;
    size_t cert_len;
    const char *client_cert_pem;
    size_t client_cert_len;
    const char *client_key_pem;
    size_t client_key_len;
    esp_http_client_method_t method;
    int timeout_ms;
    bool disable_auto_redirect;
    int max_redirection_count;
    http_event_handle_cb event_handler;
    esp_http_client_transport_t transport_type;
    int buffer_size;
    int buffer_size_tx;
    void *user_data;
    bool is_async;
    bool use_global_ca_store;
    bool skip_cert_common_name_check;
    bool (*crt_bundle_attach)(void*);
    bool keep_alive_enable;
    int keep_alive_idle;
    int keep_alive_interval;
    int keep_alive_count;
} esp_http_client_config_t;

/* Forward decls — definitions in esp_stubs.c so coverage tests can poke
   _stub_http_read_count/_stub_http_content_length/_stub_http_read_data globals. */
esp_http_client_handle_t esp_http_client_init(const esp_http_client_config_t *config);
esp_err_t esp_http_client_perform(esp_http_client_handle_t client);
esp_err_t esp_http_client_cleanup(esp_http_client_handle_t client);
esp_err_t esp_http_client_open(esp_http_client_handle_t client, int write_len);
esp_err_t esp_http_client_close(esp_http_client_handle_t client);
int esp_http_client_get_status_code(esp_http_client_handle_t client);
int64_t esp_http_client_get_content_length(esp_http_client_handle_t client);
int esp_http_client_read(esp_http_client_handle_t client, char *buffer, int len);
int esp_http_client_write(esp_http_client_handle_t client, const char *buffer, int len);
esp_err_t esp_http_client_set_header(esp_http_client_handle_t client, const char *key, const char *value);
esp_err_t esp_http_client_set_url(esp_http_client_handle_t client, const char *url);
esp_err_t esp_http_client_set_method(esp_http_client_handle_t client, esp_http_client_method_t method);
esp_err_t esp_http_client_set_post_field(esp_http_client_handle_t client, const char *data, int len);
int64_t esp_http_client_fetch_headers(esp_http_client_handle_t client);
bool esp_http_client_is_complete_data_received(esp_http_client_handle_t client);
#define HttpStatus_Ok 200
