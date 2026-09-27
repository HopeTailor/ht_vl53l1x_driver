#pragma once 

#include "ht_i2c.h"

#define VL53L1X_ADDR 0x29

typedef struct {
    i2c_port_t port;
} ht_vl53l1x_dev_t;

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev, i2c_port_t port);
