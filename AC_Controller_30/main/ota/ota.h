#ifndef OTA_H
#define OTA_H

#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"

esp_err_t ota_start(void);
esp_err_t ota_write_chunk(const uint8_t *data, size_t len);
esp_err_t ota_end_and_set_boot(void);
const esp_partition_t* ota_get_target_partition(void);
void print_ota_status(void);

extern size_t ota_write_offset;
extern const esp_partition_t *ota_partition;
extern esp_ota_handle_t ota_handle;
#endif
