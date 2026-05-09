/*
 * Zigbee HA Temperature + Humidity Sensor (End Device)
 * Target: ESP32-C6
 *
 * Reports simulated data now; replace sensor_read() with AHT20-F driver later.
 */
#pragma once

#include "esp_zigbee_core.h"

/* Zigbee network config */
#define INSTALLCODE_POLICY_ENABLE   false
#define SENSOR_ENDPOINT             1
/* Channel 13 — must match zigbee2mqtt's permit_join channel */
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1l << 11)

/* Manufacturer info reported to zigbee2mqtt */
#define ESP_MANUFACTURER_NAME  "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER   "\x07"CONFIG_IDF_TARGET

/*
 * End Device config:
 *   ed_timeout  — how long coordinator keeps the device entry without a keepalive
 *   keep_alive  — how often (ms) the ED polls the coordinator
 */
#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                      \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,           \
            .keep_alive = 3000,                                     \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                               \
    {                                                               \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                                \
    {                                                               \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }
