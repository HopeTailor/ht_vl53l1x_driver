#pragma once 

#include "ht_i2c.h"

#define VL53L1X_ADDR 0x29

typedef struct {
    i2c_port_t port;
    uint8_t distance_mode;
    uint16_t timing_budget_ms;
    uint32_t inter_measurement_ms;
} ht_vl53l1x_dev_t;

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready);

esp_err_t ht_vl53l1x_get_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance);

esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev);

static esp_err_t ht_vl53l1x_set_distance_mode(ht_vl53l1x_dev_t *dev);

static esp_err_t ht_vl53l1x_set_timing_budget(ht_vl53l1x_dev_t *dev);

static esp_err_t ht_vl53l1x_set_inter_measurement(ht_vl53l1x_dev_t *dev);





