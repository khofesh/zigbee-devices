/*
 * Zigbee HA Temperature + Humidity Sensor (End Device)
 * Target: ESP32-C6
 *
 * Device profile : HA Temperature Sensor (0x0302)
 * Clusters       : Basic (0x0000), Identify (0x0003),
 *                  Temperature Measurement (0x0402),
 *                  Relative Humidity Measurement (0x0405)
 *
 * Data flow:
 *   sensor_read()  →  esp_zb_zcl_set_attribute_val()
 *                  →  ZCL attribute reporting  →  zigbee2mqtt
 *
 * To add the real AHT20-F sensor later:
 *   1. Add the AHT20 I2C driver component
 *   2. Replace the body of sensor_read() with actual I2C reads
 *   3. No other changes needed
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "esp_zb_temp_sensor.h"

/* This file must be compiled with ZB_ED_ROLE (set via CONFIG_ZB_ZED=y) */
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig (Component config -> Zboss -> Zigbee End Device)
#endif

static const char *TAG = "ZB_TEMP_SENSOR";

/* How often the sensor task updates and reports attribute values */
#define SENSOR_UPDATE_INTERVAL_MS   10000

/* --------------------------------------------------------------------------
 * Simulated sensor readings
 * Replace this struct + sensor_read() with real AHT20-F I2C calls later.
 * -------------------------------------------------------------------------- */
typedef struct {
    int16_t  temperature_cdeg;  /* °C × 100,  e.g. 2500 = 25.00 °C  */
    uint16_t humidity_cpct;     /* %RH × 100, e.g. 6000 = 60.00 %RH */
} sensor_data_t;

static sensor_data_t s_sensor = {
    .temperature_cdeg = 2500,
    .humidity_cpct    = 6000,
};

static void sensor_read(sensor_data_t *out)
{
    /*
     * Simulate a gentle drift so zigbee2mqtt graphs show live movement.
     *
     * TODO: replace with AHT20-F driver:
     *   aht20_read_temperature_humidity(&aht20_dev, &out->temperature_cdeg, &out->humidity_cpct);
     */
    s_sensor.temperature_cdeg += (int16_t)((rand() % 3) - 1);  /* ±0.01 °C  */
    s_sensor.humidity_cpct    += (uint16_t)((rand() % 5) - 2); /* ±0.02 %RH */

    *out = s_sensor;
}

/* --------------------------------------------------------------------------
 * ZCL attribute reporting setup
 * Called once after we join (or rejoin) the network.
 * -------------------------------------------------------------------------- */
static void configure_attribute_reporting(void)
{
    /*
     * min_interval: don't send more often than this (seconds)
     * max_interval: always send at least this often (seconds)
     * delta:        send immediately when change exceeds this value
     *
     * Temperature delta unit: 0.01 °C  →  10 = 0.10 °C
     * Humidity    delta unit: 0.01 %   → 100 = 1.00 %
     */
    esp_zb_zcl_reporting_info_t temp_report = {
        .direction  = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep         = SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id    = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .u.send_info = {
            .min_interval = 5,
            .max_interval = 60,
            .delta.s16    = 10,
        },
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .manuf_code     = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&temp_report);

    esp_zb_zcl_reporting_info_t hum_report = {
        .direction  = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep         = SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id    = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .u.send_info = {
            .min_interval = 5,
            .max_interval = 60,
            .delta.u16    = 100,
        },
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .manuf_code     = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&hum_report);

    ESP_LOGI(TAG, "Attribute reporting configured (temp + humidity, max 60 s interval)");
}

/* --------------------------------------------------------------------------
 * Sensor update task — runs forever, updates ZCL attributes every 10 s.
 * The ZCL reporting engine decides when to actually transmit over-the-air.
 * -------------------------------------------------------------------------- */
static void sensor_task(void *pvParameters)
{
    sensor_data_t data;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS));

        sensor_read(&data);

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(
            SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            &data.temperature_cdeg, false);
        esp_zb_zcl_set_attribute_val(
            SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            &data.humidity_cpct, false);
        esp_zb_lock_release();

        ESP_LOGI(TAG, "Updated attributes — temp: %.2f °C  humidity: %.2f %%",
                 data.temperature_cdeg / 100.0f,
                 data.humidity_cpct    / 100.0f);
    }
}

/* --------------------------------------------------------------------------
 * Start the sensor task only once (guards against duplicate starts).
 * -------------------------------------------------------------------------- */
static void start_sensor_task(void)
{
    static bool started = false;
    if (!started) {
        started = true;
        configure_attribute_reporting();
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    }
}

/* Wrapper with void return to satisfy esp_zb_callback_t signature */
static void bdb_start_steering_cb(uint8_t param)
{
    esp_zb_bdb_start_top_level_commissioning(param);
}

/* --------------------------------------------------------------------------
 * Zigbee stack signal handler
 * -------------------------------------------------------------------------- */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status != ESP_OK) {
            /* Corrupted NVRAM (e.g. after reflash) — wipe zb_storage and reboot */
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (%s), factory reset", esp_err_to_name(err_status));
            esp_zb_factory_reset();
            break;
        }
        ESP_LOGI(TAG, "Device started (%s factory-reset mode)",
                 esp_zb_bdb_is_factory_new() ? "" : "non ");
        if (esp_zb_bdb_is_factory_new()) {
            /* No saved network: find zigbee2mqtt coordinator and join */
            ESP_LOGI(TAG, "Starting network steering (looking for coordinator)...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* Saved credentials: stack auto-rejoins, we can start reporting */
            ESP_LOGI(TAG, "Rejoined saved network");
            start_sensor_task();
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ext_pan_id;
            esp_zb_get_extended_pan_id(ext_pan_id);
            ESP_LOGI(TAG,
                "Joined network — PAN ID: 0x%04hx  Channel: %d  Short addr: 0x%04hx",
                esp_zb_get_pan_id(),
                esp_zb_get_current_channel(),
                esp_zb_get_short_address());
            start_sensor_task();
        } else {
            /* Coordinator not found or busy — retry in 1 s */
            ESP_LOGW(TAG, "Network steering failed (%s), retrying...",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(bdb_start_steering_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/* --------------------------------------------------------------------------
 * Zigbee task — init stack, register device, run main loop
 * -------------------------------------------------------------------------- */
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Build cluster list for the sensor endpoint */
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_cluster_list_add_basic_cluster(cluster_list,
        esp_zb_basic_cluster_create(&basic_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(&identify_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*
     * Temperature: int16_t in 0.01 °C
     *   0x8000 = "invalid / not yet measured" per ZCL spec
     *   range: -40.00 … 85.00 °C  (covers AHT20-F spec)
     */
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = (int16_t)0x8000,
        .min_value      = -4000,
        .max_value      =  8500,
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list,
        esp_zb_temperature_meas_cluster_create(&temp_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*
     * Humidity: uint16_t in 0.01 %RH
     *   0xFFFF = "invalid / not yet measured" per ZCL spec
     *   range: 0.00 … 100.00 %RH  (covers AHT20-F spec)
     */
    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .measured_value = 0xFFFF,
        .min_value      = 0,
        .max_value      = 10000,
    };
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list,
        esp_zb_humidity_meas_cluster_create(&humidity_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Register the endpoint */
    esp_zb_endpoint_config_t ep_config = {
        .endpoint       = SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier  = ESP_MODEL_IDENTIFIER,
    };
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, SENSOR_ENDPOINT, &info);

    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* --------------------------------------------------------------------------
 * Entry point
 * -------------------------------------------------------------------------- */
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
