#pragma once

#include "stdint.h"
#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t ht_i2c_init(i2c_port_t port, int sda_pin, int scl_pin, uint32_t clk_speed);

esp_err_t ht_i2c_write_reg16(i2c_port_t port, uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, size_t len);

esp_err_t ht_i2c_read_reg16(i2c_port_t port, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, size_t len);









