#include "ht_i2c.h"
#include "ht_vl53l1x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_MASTER_SDA     21
#define I2C_MASTER_SCL     22
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "VL53L1X";

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = ht_i2c_bus_init(I2C_MASTER_SDA, I2C_MASTER_SCL, &bus_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return;
    }

    ht_i2c_scan(bus_handle);  

    i2c_master_dev_handle_t vl_handle;
    err = ht_i2c_add_device(bus_handle, VL53L1X_ADDR, I2C_MASTER_FREQ_HZ, &vl_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Add device failed: %s", esp_err_to_name(err));
        return;
    }

    uint8_t id[2] = {0, 0};
    err = ht_i2c_read_reg16(vl_handle, 0x010F, id, 2);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "ID read failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Model ID: 0x%02X, module type: 0x%02X (expected 0xEA / 0xCC)", id[0], id[1]);
    if(id[0] != 0xEA) {
        ESP_LOGE(TAG, "Not a VL53L1X, stopping.");
        return;
    }

    ht_vl53l1x_dev_t dev = {
        .i2c_dev              = vl_handle,
        .distance_mode        = VL53L1X_DIST_LONG,
        .timing_budget_ms     = 50,
        .inter_measurement_ms = 100,
        .roi_width            = 16,
        .roi_height           = 16,
        .roi_center           = 199
    };

    err = ht_vl53l1x_init(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Sensor initialized.");

    err = ht_vl53l1x_start_ranging(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Start ranging failed: %s", esp_err_to_name(err));
        return;
    }

    while (1) {
        uint8_t ready = 0;
        err = ht_vl53l1x_check_data_ready(&dev, &ready);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Data-ready check failed: %s", esp_err_to_name(err));
        } 
        else if(ready) {
            uint16_t distance_mm = 0;
            uint8_t status = 255;

            esp_err_t e1 = ht_vl53l1x_get_distance(&dev, &distance_mm);
            esp_err_t e2 = ht_vl53l1x_get_range_status(&dev, &status);
            ht_vl53l1x_clear_interrupt(&dev);

            if (e1 == ESP_OK && e2 == ESP_OK) {
                ESP_LOGI(TAG, "Distance: %u mm | Status: %u (0 = valid)", distance_mm, status);
            } else {
                ESP_LOGE(TAG, "Read failed: dist=%s status=%s", esp_err_to_name(e1), esp_err_to_name(e2));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}