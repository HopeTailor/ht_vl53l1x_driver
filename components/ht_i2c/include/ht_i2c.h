#pragma once

#include "stdint.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t ht_i2c_bus_init(int sda_pin, int scl_pin, i2c_master_bus_handle_t *bus_handle);

esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t *bus_handle, uint8_t dev_addr, uint32_t clk_speed, i2c_master_dev_handle_t *dev_handle);

esp_err_t ht_i2c_write_reg16(i2c_master_dev_handle_t *dev_handle, uint16_t dev_addr, uint16_t reg_addr, const uint8_t *data, size_t len);

esp_err_t ht_i2c_read_reg16(i2c_master_dev_handle_t *dev_handle, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, size_t len);









