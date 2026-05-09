# HA Temperature + Humidity Sensor

Zigbee End Device that reads temperature and humidity from an **ASAIR AHT20-F**
sensor and reports them over Zigbee to a coordinator (e.g. zigbee2mqtt).

Target board: **FireBeetle 2 ESP32-C6**

---

## Hardware

### AHT20-F wiring

| AHT20-F pin | FireBeetle 2 ESP32-C6 pin | GPIO |
| ----------- | ------------------------- | ---- |
| VDD         | 3V3                       | —    |
| GND         | GND                       | —    |
| SDA         | D4                        | 19   |
| SCL         | D5                        | 20   |

Software pull-ups on SDA/SCL are enabled in the I2C driver configuration.
For reliable operation — especially with long wires or multiple devices on the
bus — add external **4.7 kΩ** pull-up resistors from SDA and SCL to 3V3.

### Zigbee channel

The channel mask in `esp_zb_temp_sensor.h` is set to channel 11 by default:

```c
#define ESP_ZB_PRIMARY_CHANNEL_MASK  (1l << 11)
```

Change this to match the channel used by your zigbee2mqtt coordinator.

---

## Architecture

```
app_main()
  └─ esp_zb_task  (FreeRTOS task, priority 5)
       ├─ esp_zb_init() + cluster/endpoint registration
       └─ esp_zb_stack_main_loop()   ← runs forever
            └─ esp_zb_app_signal_handler()
                 ├─ SKIP_STARTUP  → BDB initialization
                 ├─ DEVICE_FIRST_START / DEVICE_REBOOT
                 │    ├─ OK + factory new  → network steering
                 │    ├─ OK + saved creds  → start_sensor_task()
                 │    └─ FAIL             → esp_zb_factory_reset() + reboot
                 └─ BDB_SIGNAL_STEERING
                      ├─ OK    → start_sensor_task()
                      └─ FAIL  → scheduler alarm → retry steering in 1 s

start_sensor_task()   (called once after joining)
  ├─ configure_attribute_reporting()
  └─ sensor_task  (FreeRTOS task, priority 5)
       ├─ aht20_init()         — 500 ms power-on delay + calibration check
       ├─ xSemaphoreCreateBinary()
       ├─ esp_timer_create()   — ESP_TIMER_ISR dispatch
       ├─ esp_timer_start_periodic()  — fires every SENSOR_UPDATE_INTERVAL_MS
       └─ loop:
            xSemaphoreTake()           ← blocks here
            aht20_read_temperature_humidity()
            esp_zb_zcl_set_attribute_val() × 2
```

---

## Interrupt-driven measurement

The sensor task does **not** use `vTaskDelay` for timing. Instead:

```
SYSTIMER hardware interrupt
        │
        ▼  (ESP_TIMER_ISR dispatch — runs directly in ISR context)
sensor_timer_isr_cb()
        │
        └─► xSemaphoreGiveFromISR()  +  portYIELD_FROM_ISR()
                    │
                    ▼
            sensor_task unblocks
                    │
                    └─► aht20_read_temperature_humidity()
                        esp_zb_zcl_set_attribute_val()
```

Key points:

- `CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y` (set in
  `sdkconfig.defaults`) enables `ESP_TIMER_ISR` dispatch. Without it,
  `esp_timer` callbacks run in a helper task, not in the ISR itself.

- The callback (`sensor_timer_isr_cb`) is marked `IRAM_ATTR` so it lives in
  instruction RAM and can execute even when the flash cache is temporarily
  disabled (e.g. during a flash write).

- Inside the ISR, only `xSemaphoreGiveFromISR` is called — no I2C, no
  logging, no heap allocation. All real work happens in `sensor_task`.

- `portYIELD_FROM_ISR()` is called when `xSemaphoreGiveFromISR` sets
  `pxHigherPriorityTaskWoken = pdTRUE`, causing an immediate context switch
  to `sensor_task` on ISR exit rather than waiting for the next FreeRTOS tick.

### Why not `vTaskDelay`?

| `vTaskDelay(10 000 ms)`                   | Timer ISR + semaphore            |
| ----------------------------------------- | -------------------------------- |
| Drifts: each `vTaskDelay` starts _after_  | Fires at an absolute period from |
| the previous measurement finishes.        | the timer creation point.        |
| Period = delay + I2C read time (~100 ms). | Period is exactly the configured |
|                                           | interval regardless of I2C time. |
| Consumes a FreeRTOS timer slot while      | Task is truly blocked (no timer  |
| blocked.                                  | slot consumed).                  |

---

## ZCL attribute values

| Cluster                 | Attribute     | ZCL unit | AHT20-F range |
| ----------------------- | ------------- | -------- | ------------- |
| Temperature Measurement | MeasuredValue | 0.01 °C  | −4000 … 8500  |
| Relative Humidity       | MeasuredValue | 0.01 %RH | 0 … 10000     |

Initial values (`0x8000` for temperature, `0xFFFF` for humidity) mean
"not yet measured" per the ZCL specification. They are replaced with real
readings after the first timer tick (~10 s after joining).

Humidity is computed from the 20-bit raw ADC value at full 0.01 % resolution:

```c
uint16_t hum_zcl = (uint16_t)((float)hum_raw / 1048576.0f * 10000.0f);
```

This avoids the integer truncation in the LibDriver's `uint8_t humidity_s`
output.

---

## ZCL attribute reporting

Reporting is configured in `configure_attribute_reporting()`:

| Parameter    | Temperature | Humidity |
| ------------ | ----------- | -------- |
| min_interval | 5 s         | 5 s      |
| max_interval | 60 s        | 60 s     |
| delta        | 0.10 °C     | 1.00 %   |

The ZCL engine sends a report when either the delta threshold is exceeded
(within the min/max window) or when `max_interval` has elapsed with no report.

---

## NVRAM and factory reset

If the `zb_storage` partition contains incompatible data (e.g. after flashing
new firmware), `DEVICE_FIRST_START` returns `ESP_FAIL`. The signal handler
calls `esp_zb_factory_reset()`, which erases the Zigbee NVRAM and reboots.
The second boot will find a clean partition and proceed normally.

To force a clean start manually:

```bash
idf.py -p /dev/ttyUSBx erase-flash
idf.py -p /dev/ttyUSBx flash monitor
```

---

## Build and flash

```bash
# Source the ESP-IDF environment once per shell session
. $IDF_PATH/export.sh

cd HA_temperature_sensor

# First build (generates sdkconfig from sdkconfig.defaults)
idf.py build

# Flash and open serial monitor
idf.py -p /dev/ttyUSBx flash monitor
```

---

## Pairing with zigbee2mqtt

1. Open `zigbee2mqtt` permit_join (via Home Assistant UI or MQTT topic
   `zigbee2mqtt/bridge/request/permit_join`).
2. Power-cycle or reset the device.
3. The device will perform network steering and appear in zigbee2mqtt within
   ~30 s.
4. It exposes two sensors: `temperature` (°C) and `humidity` (%).

---

## File structure

```
main/
  esp_zb_temp_sensor.c   Main application: Zigbee stack + sensor task
  esp_zb_temp_sensor.h   Project-wide defines (endpoint, channel, ZED config)
  aht20_hal.c            ESP-IDF I2C HAL callbacks for the LibDriver
  aht20_hal.h            HAL public interface
  driver_aht20.c         LibDriver AHT20 core (© Shifeng Li, MIT licence)
  driver_aht20.h         LibDriver AHT20 header
  CMakeLists.txt
partitions.csv           Custom partition table (nvs, phy, factory, zb_storage, zb_fct)
sdkconfig.defaults       Kconfig defaults (ZED role, custom partitions, ISR timer)
```
