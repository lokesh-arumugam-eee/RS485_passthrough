#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef void* esp_ota_handle_t;
typedef enum { ESP_PARTITION_TYPE_APP=0, ESP_PARTITION_TYPE_DATA=1 } esp_partition_type_t;
typedef enum { ESP_PARTITION_SUBTYPE_APP_FACTORY=0, ESP_PARTITION_SUBTYPE_APP_OTA_0=0x10, ESP_PARTITION_SUBTYPE_ANY=0xFF } esp_partition_subtype_t;
typedef struct {
    esp_partition_type_t type;
    esp_partition_subtype_t subtype;
    uint32_t address;
    uint32_t size;
    char label[16];
    bool encrypted;
} esp_partition_t;
typedef void* esp_partition_iterator_t;
/* Forward decls — definitions in esp_stubs.c so coverage tests can drive the
   partition iteration loop in ota_diag with a real iterator sequence. */
extern int _stub_partition_iter_count; /* number of iterations to yield */
esp_partition_iterator_t esp_partition_find(esp_partition_type_t type, esp_partition_subtype_t subtype, const char *label);
const esp_partition_t* esp_partition_get(esp_partition_iterator_t iterator);
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t iterator);
void esp_partition_iterator_release(esp_partition_iterator_t iterator);
static inline const esp_partition_t* esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype, const char *label){(void)type;(void)subtype;(void)label;return NULL;}

/* Forward decls — definitions in esp_stubs.c so coverage tests can poke
   _stub_ota_running_null/_stub_ota_next_null knobs to drive NULL-partition
   error paths in firmware. */
const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t *start);
const esp_partition_t* esp_ota_get_running_partition(void);
const esp_partition_t* esp_ota_get_boot_partition(void);
esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle);
esp_err_t esp_ota_write(esp_ota_handle_t handle, const void *data, size_t size);
esp_err_t esp_ota_end(esp_ota_handle_t handle);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition);
esp_err_t esp_ota_abort(esp_ota_handle_t handle);
#define OTA_SIZE_UNKNOWN 0xFFFFFFFF
#define OTA_WITH_SEQUENTIAL_WRITES 0
