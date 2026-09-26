#pragma once

#include "stdint.h"
#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t ht_i2c_init(i2c_port_t port, int sda_pin, int scl_pin, uint32_t clk_speed);