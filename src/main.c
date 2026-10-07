/**
 * @file main.c
 * @brief Application entry point for VL53L1X ToF Sensor testing.
 * 
 * Demonstrates how to initialize the I2C bus, verify sensor hardware ID,
 * configure the Ultra Lite Driver (ULD), and read distance data asynchronously
 * using FreeRTOS tasks.
 */

#include "ht_i2c.h"
#include "ht_vl53l1x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Hardware I2C Configuration */
#define I2C_MASTER_SDA     21       // I2C Data pin
#define I2C_MASTER_SCL     22       // I2C Clock pin
#define I2C_MASTER_FREQ_HZ 400000   // 400kHz (Fast Mode)

static const char *TAG = "VL53L1X";

void app_main(void) {
    // ---------------------------------------------------------
    // Step 1: Initialize the main I2C bus
    // ---------------------------------------------------------
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = ht_i2c_bus_init(I2C_MASTER_SDA, I2C_MASTER_SCL, &bus_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return;
    }

    // Scan the bus to ensure physical connections are secure
    ht_i2c_scan(bus_handle);  

    // ---------------------------------------------------------
    // Step 2: Add the ToF sensor to the I2C bus
    // ---------------------------------------------------------
    i2c_master_dev_handle_t vl_handle;
    err = ht_i2c_add_device(bus_handle, VL53L1X_ADDR, I2C_MASTER_FREQ_HZ, &vl_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Add device failed: %s", esp_err_to_name(err));
        return;
    }

    // ---------------------------------------------------------
    // Step 3: Hardware validation (prevent running on fake/wrong sensors)
    // ---------------------------------------------------------
    uint8_t id[2] = {0, 0};
    // Read register 0x010F which contains the device ID
    err = ht_i2c_read_reg16(vl_handle, 0x010F, id, 2);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "ID read failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Model ID: 0x%02X, module type: 0x%02X (expected 0xEA / 0xCC)", id[0], id[1]);
    
    // Genuine VL53L1X must return 0xEA
    if(id[0] != 0xEA) {
        ESP_LOGE(TAG, "Not a VL53L1X, stopping.");
        return;
    }

    // ---------------------------------------------------------
    // Step 4: Configure sensor parameters
    // ---------------------------------------------------------
    ht_vl53l1x_dev_t dev = {
        .i2c_dev              = vl_handle,
        .distance_mode        = VL53L1X_DIST_LONG, // Long range mode (up to 4m in dark)
        .timing_budget_ms     = 50,                // Time allocated per measurement (medium accuracy)
        .inter_measurement_ms = 100,               // Delay between measurements (MUST be >= timing_budget)
        .roi_width            = 16,                // Full Field of View width (16x16 SPADs)
        .roi_height           = 16,                // Full Field of View height
        .roi_center           = 199                // Default center of the optical receiver array
    };

    // Boot sensor and perform VHV (Very High Voltage) calibration
    err = ht_vl53l1x_init(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Sensor initialized.");

    // Start continuous ranging
    err = ht_vl53l1x_start_ranging(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Start ranging failed: %s", esp_err_to_name(err));
        return;
    }

    // ---------------------------------------------------------
    // Step 5: Main Application Loop (Non-blocking)
    // ---------------------------------------------------------
    while (1) {
        uint8_t ready = 0;
        
        // Poll the status register for new data (without blocking the CPU)
        err = ht_vl53l1x_check_data_ready(&dev, &ready);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Data-ready check failed: %s", esp_err_to_name(err));
        } 
        else if(ready) {
            uint16_t distance_mm = 0;
            uint8_t status = 255;

            // Extract distance and signal health status
            esp_err_t e1 = ht_vl53l1x_get_distance(&dev, &distance_mm);
            esp_err_t e2 = ht_vl53l1x_get_range_status(&dev, &status);
            
            // Clear interrupt flag to trigger the next measurement (CRITICAL)
            ht_vl53l1x_clear_interrupt(&dev);

            if (e1 == ESP_OK && e2 == ESP_OK) {
                // Status 0 means data is perfectly valid
                ESP_LOGI(TAG, "Distance: %u mm | Status: %u (0 = valid)", distance_mm, status);
            } else {
                ESP_LOGE(TAG, "Read failed: dist=%s status=%s", esp_err_to_name(e1), esp_err_to_name(e2));
            }
        }
        
        // 20ms delay to yield to FreeRTOS scheduler (prevents Watchdog errors)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}