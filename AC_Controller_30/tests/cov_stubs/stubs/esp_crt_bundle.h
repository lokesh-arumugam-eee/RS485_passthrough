#pragma once
#include "esp_err.h"
static inline esp_err_t esp_crt_bundle_attach(void *conf){(void)conf;return ESP_OK;}
static inline void esp_crt_bundle_detach(void *conf){(void)conf;}
static inline esp_err_t esp_crt_bundle_set(const uint8_t *x509_bundle, size_t bundle_size){(void)x509_bundle;(void)bundle_size;return ESP_OK;}
