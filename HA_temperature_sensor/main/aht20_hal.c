/*
 * ESP-IDF HAL for AHT20-F using the new I2C master driver (esp_driver_i2c).
 *
 * FireBeetle 2 ESP32-C6 pin assignment:
 *   SDA — GPIO 19
 *   SCL — GPIO 20
 *
 * The new I2C master API (ESP-IDF ≥ 5.1) works with bus and device handles
 * rather than the legacy i2c_driver_install / i2c_cmd_link approach.
 * Bus-level pull-ups are enabled in software; add 4.7 kΩ external resistors
 * for reliable operation at the default 100 kHz clock.
 */

#include "aht20_hal.h"

#include <stdarg.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG             "AHT20_HAL"

#define I2C_SDA_GPIO    19
#define I2C_SCL_GPIO    20
#define I2C_PORT_NUM    I2C_NUM_0
#define AHT20_I2C_ADDR  0x38    /* 7-bit address (LibDriver uses 0x70 = 0x38 << 1) */
#define I2C_SPEED_HZ    100000  /* 100 kHz — safe for long wires */
#define I2C_TIMEOUT_MS  100     /* per-transaction timeout */

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_aht20_dev;

uint8_t aht20_hal_i2c_init(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port              = I2C_PORT_NUM,
        .sda_io_num            = I2C_SDA_GPIO,
        .scl_io_num            = I2C_SCL_GPIO,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed");
        return 1;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT20_I2C_ADDR,
        .scl_speed_hz    = I2C_SPEED_HZ,
    };
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_aht20_dev) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed");
        i2c_del_master_bus(s_i2c_bus);
        return 1;
    }

    ESP_LOGI(TAG, "I2C master ready — SDA GPIO%d  SCL GPIO%d  speed %d Hz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_SPEED_HZ);
    return 0;
}

uint8_t aht20_hal_i2c_deinit(void)
{
    i2c_master_bus_rm_device(s_aht20_dev);
    i2c_del_master_bus(s_i2c_bus);
    return 0;
}

uint8_t aht20_hal_i2c_read(uint8_t addr, uint8_t *buf, uint16_t len)
{
    (void)addr; /* address is baked into the device handle */
    if (i2c_master_receive(s_aht20_dev, buf, len, I2C_TIMEOUT_MS) != ESP_OK) {
        return 1;
    }
    return 0;
}

uint8_t aht20_hal_i2c_write(uint8_t addr, uint8_t *buf, uint16_t len)
{
    (void)addr;
    if (i2c_master_transmit(s_aht20_dev, buf, len, I2C_TIMEOUT_MS) != ESP_OK) {
        return 1;
    }
    return 0;
}

void aht20_hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void aht20_hal_debug_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    esp_log_writev(ESP_LOG_WARN, TAG, fmt, args);
    va_end(args);
}
