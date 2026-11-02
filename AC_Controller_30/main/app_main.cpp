/*
 * Matter on/off-plug PoC for AC_Controller_30.
 *
 * Minimal bring-up: create a Matter node with the mandatory Root Node on
 * endpoint 0, add a single On/Off Plug-in Unit endpoint, and start the
 * Matter stack (Wi-Fi + NimBLE commissioning handled by esp-matter).
 * No GPIO/LED/button driver is wired up — on/off changes are just logged —
 * so this stays a pure proof-of-concept for build size and commissioning.
 */

#include <inttypes.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>

static const char *TAG = "ac_matter";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

/* Called on every attribute write. For the PoC we just log On/Off changes. */
static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == PRE_UPDATE) {
        ESP_LOGI(TAG, "attr update: endpoint=%u cluster=0x%08" PRIx32 " attribute=0x%08" PRIx32,
                 endpoint_id, cluster_id, attribute_id);
    }
    return ESP_OK;
}

/* Invoked when a controller asks the device to identify itself. */
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id,
                                       uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "identify: type=%u effect=%u", type, effect_id);
    return ESP_OK;
}

/* Matter stack lifecycle events. */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP address changed");
        break;
    default:
        break;
    }
}

extern "C" void app_main(void)
{
    /* NVS is required by the Matter stack (fabrics, keys, etc.). */
    nvs_flash_init();

    /* Root Node device type on endpoint 0 is created with the node. */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return;
    }

    /* Single On/Off Plug-in Unit endpoint. */
    on_off_plug_in_unit::config_t plug_config;
    plug_config.on_off.on_off = false;
    endpoint_t *endpoint = on_off_plug_in_unit::create(node, &plug_config, ENDPOINT_FLAG_NONE, NULL);
    if (!endpoint) {
        ESP_LOGE(TAG, "Failed to create on/off plug endpoint");
        return;
    }
    ESP_LOGI(TAG, "On/Off plug endpoint created: id=%u", endpoint::get_id(endpoint));

    /* Start Matter (brings up Wi-Fi/BLE commissioning and the data model). */
    esp_err_t err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter: %d", err);
        return;
    }

    ESP_LOGI(TAG, "Matter on/off-plug PoC started");
}
