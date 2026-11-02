#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include <inttypes.h>
static const char *TAG = "OTA";
size_t ota_write_offset = 0;
const esp_partition_t *ota_partition;
esp_ota_handle_t ota_handle;

// Get the partition to update (non-running one)
const esp_partition_t *ota_get_target_partition(void)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        ESP_LOGE(TAG, "No valid OTA target partition found!");
    }
    return target;
}

// Start OTA
esp_err_t ota_start(void)
{
    const esp_partition_t *target = ota_get_target_partition();
    if (!target) return ESP_FAIL;

    esp_err_t err = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return err;
    }

    ota_write_offset = 0;
    ESP_LOGI(TAG, "OTA started on partition %s", target->label);
    return ESP_OK;
}

// Write a chunk of OTA data
esp_err_t ota_write_chunk(const uint8_t *data, size_t len)
{
    if (ota_handle == 0) {
        ESP_LOGE(TAG, "OTA handle not initialized");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s — aborting OTA session",
                 esp_err_to_name(err));
        /* BUG-FIX #9: previously left ota_handle valid and ota_write_offset
           at its pre-failure value. Subsequent writes would proceed against
           a corrupted partition; a reboot mid-OTA could land on a partial
           image that passed the IDF header check and boot-looped the unit.
           Abort the session and clear state so the next ota_start() begins
           cleanly. */
        esp_ota_abort(ota_handle);
        ota_handle = 0;
        ota_write_offset = 0;
        return err;
    }

    ota_write_offset += len;
    return ESP_OK;
}

// Finish OTA and switch boot partition
esp_err_t ota_end_and_set_boot(void)
{
    if (ota_handle == 0) {
        ESP_LOGE(TAG, "OTA handle not initialized");
        return ESP_FAIL;
    }

    // End OTA write
    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return err;
    }

    // Get target partition again
    const esp_partition_t *target = ota_get_target_partition();
    if (!target) return ESP_FAIL;

    // Set boot partition
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA finished. Next boot from partition %s", target->label);
    return ESP_OK;
}

void print_ota_status(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    if (running) {
        ESP_LOGI(TAG, "Running from: %s type=0x%x,subtype=0x%x,addr=0x%x,size=%" PRIu32,running->label, (unsigned int)running->type, (unsigned int)running->subtype,
                (unsigned int)running->address, running->size);
    } else {
        ESP_LOGE(TAG, "Unable to determine running partition!");
    }

    if (next) {
       ESP_LOGI(TAG, "Next OTA target: %s type=0x%x,subtype=0x%x,addr=0x%x,size=%" PRIu32,next->label,(unsigned int) next->type,(unsigned int) next->subtype,
         (unsigned int) next->address,next->size);
    } else {
        ESP_LOGE(TAG, "No valid OTA target found!");
    }
}
