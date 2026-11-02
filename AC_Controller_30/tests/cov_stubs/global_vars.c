/*
 * global_vars.c — Provides ONLY the symbols that are declared extern
 * in project headers but are NOT defined in any compiled .c source file.
 *
 * 1. Certificate binary assets:
 *    mqtt_manager.h (with HTTPS_CONNECTION==2) declares:
 *      extern const uint8_t server_cert_pem_start[]
 *        asm("_binary_certificate_crt_start");
 *      extern const uint8_t server_cert_pem_end[]
 *        asm("_binary_certificate_crt_end");
 *    The linker looks for _binary_certificate_crt_start/_end (the asm alias).
 *    We define stub arrays with those linker names here.
 *
 * 2. rtu_platform_conf:
 *    modbus_rtu.h declares `extern nmbs_platform_conf rtu_platform_conf;`
 *    but modbus_rtu.c defines a *different* variable `nmbs_platform_conf rtu_conf`.
 *    Provide a zero-initialised instance to satisfy the linker.
 */

#include <stdint.h>
#include <stddef.h>

/* ---- Certificate stubs (normally injected by esp-idf CMake linker) -------- */
/* GCC asm() attribute gives these the exact linker symbol names expected.     */
const uint8_t _binary_certificate_crt_start[1]
    __attribute__((visibility("default"))) = {0};

const uint8_t _binary_certificate_crt_end[1]
    __attribute__((visibility("default"))) = {0};

/* Also provide the JIO_MQTT variant in case MQTT_DETAILS is changed */
const uint8_t _binary_gd_bundle_g2_crt_start[1]
    __attribute__((visibility("default"))) = {0};

const uint8_t _binary_gd_bundle_g2_crt_end[1]
    __attribute__((visibility("default"))) = {0};

/* ---- nanomodbus platform config ------------------------------------------- */
#include "nanomodbus.h"
nmbs_platform_conf rtu_platform_conf;
