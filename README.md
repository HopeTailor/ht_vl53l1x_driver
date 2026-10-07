# Professional VL53L1X Time-of-Flight Driver for ESP-IDF (`ht_vl53l1x`)

A robust, non-blocking, industrial-grade C driver for the STMicroelectronics VL53L1X Time-of-Flight (ToF) distance sensor, built specifically for the Espressif ESP-IDF framework and FreeRTOS.

Unlike basic Arduino ports, this library is architected around ST's official Ultra Lite Driver (ULD) logic. It handles the undocumented "magic number" registers, dynamic PLL clock synchronization, and hardware interrupt polarity detection to provide highly stable, non-blocking distance measurements suitable for advanced robotics and embedded systems.

## 🚀 Features
* **True Non-Blocking Architecture:** Uses hardware interrupt registers (`GPIO_TIO_HV_STATUS`) to poll for data readiness, keeping your FreeRTOS tasks unblocked.
* **ST ULD Compliant:** Implements the official 91-byte configuration array and VHV (Very High Voltage) calibration sequence required for reliable laser firing.
* **Dynamic PLL Sync:** Calculates the internal oscillator calibration value to perfectly sync the ESP32 I2C reads with the sensor's inter-measurement periods.
* **Custom Field of View (ROI):** Shrink the laser's Region of Interest (ROI) to prevent ground reflections in wheeled robots.
* **Hardware Validation:** Automatically verifies the `0xEA` hardware ID to prevent crashes on counterfeit or mismatched sensors (like the VL53L0X).

## 🔌 Wiring & Hardware Setup

| VL53L1X Pin | ESP32 Pin (Default) | Notes |
| :--- | :--- | :--- |
| **VIN / VCC** | 3.3V | *Do NOT connect to 5V unless your module has a built-in regulator.* |
| **GND** | GND | Common ground. |
| **SCL** | GPIO 22 | I2C Clock (Requires pull-up resistor if not on breakout board). |
| **SDA** | GPIO 21 | I2C Data (Requires pull-up resistor if not on breakout board). |
| **XSHUT** | Unconnected | Optional: Pull low to put the sensor in hardware standby. |

> **⚠️ CRITICAL HARDWARE NOTE:** Brand new VL53L1X modules often ship with a microscopic, amber-colored piece of Kapton tape covering the laser aperture. **You MUST remove this tape with tweezers before use.** If left on, the sensor will initialize successfully but constantly report `0 mm` distance.

## 💻 Quick Start Usage

The library relies on a custom `ht_i2c` abstraction layer for modern ESP-IDF (v5.0+) handle-based I2C management.

```c
#include "ht_i2c.h"
#include "ht_vl53l1x.h"

void app_main(void) {
    // 1. Initialize I2C Master
    i2c_master_bus_handle_t bus_handle;
    ht_i2c_bus_init(21, 22, &bus_handle);

    // 2. Add Sensor to Bus
    i2c_master_dev_handle_t vl_handle;
    ht_i2c_add_device(bus_handle, VL53L1X_ADDR, 400000, &vl_handle);

    // 3. Configure the Sensor
    ht_vl53l1x_dev_t dev = {
        .i2c_dev              = vl_handle,
        .distance_mode        = VL53L1X_DIST_LONG,
        .timing_budget_ms     = 50,
        .inter_measurement_ms = 100,
        .roi_width            = 16,
        .roi_height           = 16,
        .roi_center           = 199
    };

    ht_vl53l1x_init(&dev);
    ht_vl53l1x_start_ranging(&dev);

    // 4. Non-blocking Read Loop
    while (1) {
        uint8_t ready = 0;
        ht_vl53l1x_check_data_ready(&dev, &ready);
        
        if(ready) {
            uint16_t distance_mm = 0;
            uint8_t status = 255;

            ht_vl53l1x_get_distance(&dev, &distance_mm);
            ht_vl53l1x_get_range_status(&dev, &status);
            ht_vl53l1x_clear_interrupt(&dev); // MUST be called to trigger next fire

            if (status == 0) {
                printf("Target found at: %u mm\n", distance_mm);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

## ⚙️ Advanced Configuration

### Distance Modes
*   `VL53L1X_DIST_SHORT`: Maximum range ~1.3m. More robust against ambient light (sunlight) and offers a slightly higher refresh rate capability.
*   `VL53L1X_DIST_LONG`: Maximum range ~4.0m. Highly sensitive to ambient light. Requires a completely dark environment to hit the absolute 4m max range reliably.

### Region of Interest (ROI)
The receiver utilizes a 16x16 SPAD (Single Photon Avalanche Diode) array. By reducing `roi_width` and `roi_height` (Minimum: 4x4), you narrow the field of view. This is crucial for ground-based robots to prevent the lower edge of the laser cone from reflecting off the floor and generating false "close target" readings.

## 🛠️ Troubleshooting & Status Codes

If the sensor returns a status code other than `0` (Valid), refer to this breakdown:

*   **Status `1` or `2` (Signal Fail):** The target is too unreflective (e.g., black fabric) or the laser is being drowned out by ambient sunlight.
*   **Status `4` (Out of Bounds):** The target is completely out of range, or the laser is shooting into open sky.
*   **Distance is always `0 mm` (Status `0`):** You forgot to remove the protective Kapton tape from the sensor aperture.
*   **Initialization hangs / I2C Error:** Ensure SDA and SCL are strictly connected to 3.3V pull-up resistors (typically 4.7kΩ).

## 📚 Acknowledgments & Inspiration

This driver was engineered from the ground up for ESP-IDF, but its mathematical backbone and register tuning strategies were deeply inspired by the following open-source resources and documentation:

*   **STMicroelectronics UM2510 Manual:** The official user manual and specification for the VL53L1X Ultra Lite Driver (ULD) API, which dictated the VHV calibration sequences and PLL formulas.
*   **Pololu VL53L1X Arduino Library:** Pololu's exceptional reverse-engineering of the ST API provided clear insights into the 91-byte default configuration array and the complex Phase/Timeout register mappings.
*   **Pimoroni VL53L1X Python Library:** Served as an excellent reference for handling register boundaries and testing ROI configurations in embedded environments.