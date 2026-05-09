/*
 * ESP-IDF hardware abstraction layer for the LibDriver AHT20-F sensor.
 *
 * Provides the four function pointers required by aht20_handle_t:
 *   iic_init / iic_deinit / iic_read_cmd / iic_write_cmd
 * plus delay_ms and debug_print.
 *
 * Hardware: FireBeetle 2 ESP32-C6
 *   SDA — GPIO 19
 *   SCL — GPIO 20
 */
#pragma once

#include <stdint.h>

/* Install I2C master bus and add AHT20 device. */
uint8_t aht20_hal_i2c_init(void);

/* Remove AHT20 device and uninstall I2C master bus. */
uint8_t aht20_hal_i2c_deinit(void);

/*
 * Receive `len` bytes from the AHT20.
 * `addr` is the 8-bit address passed by the LibDriver (0x70); it is ignored
 * here because the device handle already carries the correct 7-bit address.
 */
uint8_t aht20_hal_i2c_read(uint8_t addr, uint8_t *buf, uint16_t len);

/* Transmit `len` bytes to the AHT20. Same note on `addr` as above. */
uint8_t aht20_hal_i2c_write(uint8_t addr, uint8_t *buf, uint16_t len);

/* Block the calling task for `ms` milliseconds using FreeRTOS. */
void aht20_hal_delay_ms(uint32_t ms);

/* Route LibDriver diagnostic messages to the ESP-IDF log system. */
void aht20_hal_debug_print(const char *fmt, ...);
