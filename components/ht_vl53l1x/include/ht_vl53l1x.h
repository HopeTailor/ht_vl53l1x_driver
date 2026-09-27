#pragma once 

#include "ht_i2c.h"

#define VL53L1X_ADDR 0x29

typedef struct {
    i2c_port_t port;
} ht_vl53l1x_dev_t;

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev, i2c_port_t port);

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready);

esp_err_t ht_vl53l1x_det_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance);

esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev);






