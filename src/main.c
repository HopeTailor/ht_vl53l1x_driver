#include "ht_i2c.h"
#include "ht_vl53l1x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "VL53L1X";

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C Master...");
    i2c_master_bus_handle_t bus_handle;

    esp_err_t err = ht_i2c_bus_init(I2C_MASTER_SDA, I2C_MASTER_SCL, &bus_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C Initialization Failed!");
        return;
    }

    ESP_LOGI(TAG, "Adding VL53L1X to I2C Bus...");
    i2c_master_dev_handle_t vl53l1x_handle;
    err = ht_i2c_add_device(bus_handle, VL53L1X_ADDR, I2C_MASTER_FREQ_HZ, &vl53l1x_handle);
    ESP_LOGI(TAG, "Configuring VL53L1X Sensor...");

    ht_vl53l1x_dev_t dev = {
        .i2c_dev = vl53l1x_handle,
        .distance_mode = 1,        
        .timing_budget_ms = 50,       
        .inter_measurement_ms = 100,  
        .roi_width = 16,              
        .roi_height = 16,             
        .roi_center = 199             
    };

    err = ht_vl53l1x_init(&dev);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X Initialization Failed! Error Code: %d", err);
        return;
    }
    ESP_LOGI(TAG, "VL53L1X Initialized Successfully!");

    err = ht_vl53l1x_start_ranging(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ranging!");
        return;
    }

    while(1) {
        uint8_t data_ready = 0;
        ht_vl53l1x_check_data_ready(&dev, &data_ready);

        if(data_ready == 1) {
            uint16_t distance_mm = 0;
            ht_vl53l1x_get_distance(&dev, &distance_mm);
            ht_vl53l1x_clear_interrupt(&dev);
            ESP_LOGI(TAG, "Distance: %u mm", distance_mm);
        }

        vTaskDelay(10);
    }
    
}
