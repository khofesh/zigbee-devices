/*
 * Zigbee HA Temperature + Humidity Sensor (End Device)
 * Target: FireBeetle 2 ESP32-C6
 *
 * Device profile : HA Temperature Sensor (0x0302)
 * Clusters       : Basic (0x0000), Identify (0x0003),
 *                  Temperature Measurement (0x0402),
 *                  Relative Humidity Measurement (0x0405)
 *
 * Sensor : ASAIR AHT20-F on I2C0 (SDA = GPIO19, SCL = GPIO20)
 *
 * Measurement flow (interrupt-driven):
 *
 *   SYSTIMER hardware interrupt
 *          │
 *          ▼
 *   esp_timer ISR callback  ──► xSemaphoreGiveFromISR()
 *                                      │
 *                                      ▼
 *                             sensor_task unblocks
 *                                      │
 *                                      ▼
 *                             aht20_read_temperature_humidity()
 *                                      │
 *                                      ▼
 *                             esp_zb_zcl_set_attribute_val()
 *                                      │
 *                                      ▼
 *                             ZCL attribute reporting ──► zigbee2mqtt
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "esp_zb_temp_sensor.h"
#include "driver_aht20.h"
#include "aht20_hal.h"

/* This file must be compiled with ZB_ED_ROLE (set via CONFIG_ZB_ZED=y) */
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig (Component config -> Zboss -> Zigbee End Device)
#endif

static const char *TAG = "ZB_TEMP_SENSOR";

/* Measurement period in milliseconds */
#define SENSOR_UPDATE_INTERVAL_MS   10000

/*
 * Status LED — onboard blue LED on FireBeetle 2 ESP32-C6 is GPIO15 (active-high).
 * Blinks once each time a measurement is successfully pushed to the Zigbee stack,
 * so you get visual confirmation that the device is alive and sending data.
 */
#define STATUS_LED_GPIO   GPIO_NUM_15
#define STATUS_LED_ON_MS  50

static void status_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << STATUS_LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(STATUS_LED_GPIO, 0);
}

/* Brief blink to signal "data sent". Runs in sensor_task context (not the ISR). */
static void status_led_blink(void)
{
    gpio_set_level(STATUS_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(STATUS_LED_ON_MS));
    gpio_set_level(STATUS_LED_GPIO, 0);
}

/*
 * Binary semaphore posted by the timer ISR and consumed by sensor_task.
 * Using a semaphore (not a direct task notification) so the ISR can use
 * the FromISR variant without knowing the task handle at creation time.
 */
static SemaphoreHandle_t s_measure_sem;

/* --------------------------------------------------------------------------
 * Timer ISR callback — runs in hardware interrupt context (SYSTIMER).
 *
 * Must be in IRAM so it executes even when flash cache is disabled.
 * Only posts the semaphore; all real work is done in sensor_task.
 * -------------------------------------------------------------------------- */
static void IRAM_ATTR sensor_timer_isr_cb(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_measure_sem, &woken);
    /* Yield to sensor_task immediately if it has higher priority than whatever
     * was running when the interrupt fired. */
    if (woken) {
        portYIELD_FROM_ISR();
    }
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
        .direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep           = SENSOR_ENDPOINT,
        .cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id      = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .u.send_info  = {
            .min_interval = 5,
            .max_interval = 60,
            .delta.s16    = 10,
        },
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .manuf_code     = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&temp_report);

    esp_zb_zcl_reporting_info_t hum_report = {
        .direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep           = SENSOR_ENDPOINT,
        .cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id      = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .u.send_info  = {
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
 * Sensor task
 *
 * Lifecycle:
 *   1. Init AHT20-F via I2C.
 *   2. Create binary semaphore.
 *   3. Create a periodic esp_timer with ISR dispatch — each expiry posts the
 *      semaphore from hardware interrupt context.
 *   4. Loop: wait on semaphore → read AHT20-F → push ZCL attributes.
 *
 * Why semaphore + ISR timer instead of vTaskDelay?
 *   - vTaskDelay ties up a FreeRTOS timer slot and drifts due to task
 *     scheduling jitter.
 *   - The SYSTIMER-backed esp_timer fires at cycle-level precision.
 *   - The task stays blocked (zero CPU) between measurements; the ISR wakes
 *     it with a single instruction, with no FreeRTOS tick dependency.
 * -------------------------------------------------------------------------- */
static void sensor_task(void *pvParameters)
{
    /* Link ESP-IDF HAL callbacks into the LibDriver handle. */
    aht20_handle_t aht20;
    DRIVER_AHT20_LINK_INIT(&aht20, aht20_handle_t);
    DRIVER_AHT20_LINK_IIC_INIT(&aht20,       aht20_hal_i2c_init);
    DRIVER_AHT20_LINK_IIC_DEINIT(&aht20,     aht20_hal_i2c_deinit);
    DRIVER_AHT20_LINK_IIC_READ_CMD(&aht20,   aht20_hal_i2c_read);
    DRIVER_AHT20_LINK_IIC_WRITE_CMD(&aht20,  aht20_hal_i2c_write);
    DRIVER_AHT20_LINK_DELAY_MS(&aht20,       aht20_hal_delay_ms);
    DRIVER_AHT20_LINK_DEBUG_PRINT(&aht20,    aht20_hal_debug_print);

    if (aht20_init(&aht20) != 0) {
        ESP_LOGE(TAG, "AHT20 init failed — check wiring (SDA GPIO19, SCL GPIO20)");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "AHT20-F initialized");

    status_led_init();

    s_measure_sem = xSemaphoreCreateBinary();
    configASSERT(s_measure_sem);

    /*
     * ESP_TIMER_ISR: the callback runs directly in the SYSTIMER interrupt
     * handler, not in a helper task.  This is the lowest-latency path and
     * requires CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y (set in
     * sdkconfig.defaults).
     */
    const esp_timer_create_args_t timer_args = {
        .callback        = sensor_timer_isr_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name            = "aht20_timer",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer,
                        (uint64_t)SENSOR_UPDATE_INTERVAL_MS * 1000ULL));
    ESP_LOGI(TAG, "Sensor timer started — period %u ms", SENSOR_UPDATE_INTERVAL_MS);

    while (1) {
        /* Block here until the ISR posts the semaphore. */
        xSemaphoreTake(s_measure_sem, portMAX_DELAY);

        uint32_t temp_raw, hum_raw;
        float    temp_s;
        uint8_t  hum_pct; /* whole-percent; we use hum_raw for ZCL precision */

        if (aht20_read_temperature_humidity(&aht20, &temp_raw, &temp_s,
                                            &hum_raw, &hum_pct) != 0) {
            ESP_LOGE(TAG, "AHT20 read failed");
            continue;
        }

        /*
         * ZCL units:
         *   Temperature Measurement (0x0402): int16_t in 0.01 °C
         *   Relative Humidity       (0x0405): uint16_t in 0.01 %RH
         *
         * hum_raw is a 20-bit fixed-point value where 2^20 = 100 %RH.
         * Multiply by 10000 before dividing to get 0.01 % resolution
         * without losing the fractional part that uint8_t hum_pct drops.
         */
        int16_t  temp_zcl = (int16_t)(temp_s * 100.0f);
        uint16_t hum_zcl  = (uint16_t)((float)hum_raw / 1048576.0f * 10000.0f);

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(
            SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            &temp_zcl, false);
        esp_zb_zcl_set_attribute_val(
            SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            &hum_zcl, false);
        esp_zb_lock_release();

        /* Visual confirmation that a measurement was pushed to the stack. */
        status_led_blink();

        ESP_LOGI(TAG, "temp: %.2f °C  humidity: %.2f %%",
                 temp_s, (float)hum_zcl / 100.0f);
    }
}

/* --------------------------------------------------------------------------
 * Start the sensor task only once (guards against duplicate starts on
 * network rejoin).
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
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (%s), factory reset",
                     esp_err_to_name(err_status));
            esp_zb_factory_reset();
            break;
        }
        ESP_LOGI(TAG, "Device started (%s factory-reset mode)",
                 esp_zb_bdb_is_factory_new() ? "" : "non ");
        if (esp_zb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Starting network steering (looking for coordinator)...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
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
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
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
     *   range: -40.00 … 85.00 °C  (AHT20-F datasheet)
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
     *   range: 0.00 … 100.00 %RH  (AHT20-F datasheet)
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
        .endpoint           = SENSOR_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
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
