#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef void* nvs_handle_t;
typedef uint32_t nvs_handle;
typedef enum { NVS_READONLY=0, NVS_READWRITE } nvs_open_mode_t;
typedef enum { NVS_TYPE_U8=0x01, NVS_TYPE_I8=0x11, NVS_TYPE_U16=0x02, NVS_TYPE_I16=0x12, NVS_TYPE_U32=0x04, NVS_TYPE_I32=0x14, NVS_TYPE_U64=0x08, NVS_TYPE_I64=0x18, NVS_TYPE_STR=0x21, NVS_TYPE_BLOB=0x42, NVS_TYPE_ANY=0xff } nvs_type_t;

/* Forward decls — definitions in esp_stubs.c so coverage tests can poke
   _stub_nvs_*_ret knobs to drive firmware error paths. */
extern esp_err_t _stub_nvs_open_ret;
extern esp_err_t _stub_nvs_commit_ret;
extern esp_err_t _stub_nvs_set_blob_ret;
extern esp_err_t _stub_nvs_get_blob_ret;
esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t *out_handle);
esp_err_t nvs_close(nvs_handle_t handle);
esp_err_t nvs_commit(nvs_handle_t handle);
static inline esp_err_t nvs_set_u8(nvs_handle_t handle, const char* key, uint8_t value){(void)handle;(void)key;(void)value;return ESP_OK;}
static inline esp_err_t nvs_get_u8(nvs_handle_t handle, const char* key, uint8_t* out_value){(void)handle;(void)key;if(out_value)*out_value=0;return ESP_OK;}
static inline esp_err_t nvs_set_i32(nvs_handle_t handle, const char* key, int32_t value){(void)handle;(void)key;(void)value;return ESP_OK;}
static inline esp_err_t nvs_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value){(void)handle;(void)key;if(out_value)*out_value=0;return ESP_OK;}
static inline esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value){(void)handle;(void)key;(void)value;return ESP_OK;}
static inline esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value, size_t* length){(void)handle;(void)key;if(out_value&&length&&*length>0)out_value[0]='\0';return ESP_OK;}
/* Forward decls — definitions in esp_stubs.c so all TUs share the same in-memory
   blob store. This lets firmware code that does set/get blob round-trip cleanly. */
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);
static inline esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key){(void)handle;(void)key;return ESP_OK;}
