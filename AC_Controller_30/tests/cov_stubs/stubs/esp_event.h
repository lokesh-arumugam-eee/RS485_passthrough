#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef const char* esp_event_base_t;
typedef void* esp_event_loop_handle_t;
typedef int32_t esp_event_id_t;
typedef void (*esp_event_handler_t)(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
typedef void* esp_event_handler_instance_t;

#define ESP_EVENT_ANY_BASE     NULL
#define ESP_EVENT_ANY_ID       (-1)
#define ESP_EVENT_DEFINE_BASE(id) esp_event_base_t id = #id
#define ESP_EVENT_DECLARE_BASE(id) extern esp_event_base_t id

typedef struct {
    int32_t queue_size;
    int32_t task_priority;
    uint32_t task_stack_size;
} esp_event_loop_args_t;

/* Coverage harness: forward declarations — definitions live in esp_stubs.c so
   all firmware translation units share the same dispatch table. */
esp_err_t esp_event_loop_create_default(void);
esp_err_t esp_event_loop_create(const esp_event_loop_args_t *event_loop_args, esp_event_loop_handle_t *event_loop);
esp_err_t esp_event_loop_delete_default(void);
esp_err_t esp_event_handler_register(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler, void *event_handler_arg);
esp_err_t esp_event_handler_register_with(esp_event_loop_handle_t event_loop, esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler, void *event_handler_arg);
esp_err_t esp_event_handler_instance_register(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler, void *event_handler_arg, esp_event_handler_instance_t *instance);
esp_err_t esp_event_handler_unregister(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler);
esp_err_t esp_event_post(esp_event_base_t event_base, int32_t event_id, void *event_data, size_t event_data_size, uint32_t ticks_to_wait);
esp_err_t esp_event_post_to(esp_event_loop_handle_t event_loop, esp_event_base_t event_base, int32_t event_id, void *event_data, size_t event_data_size, uint32_t ticks_to_wait);
