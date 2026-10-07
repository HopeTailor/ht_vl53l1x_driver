/**
 * @file ht_i2c.h
 * @brief I2C Abstraction Layer for ESP-IDF
 * 
 * Provides a clean and simplified interface for I2C bus initialization, 
 * device management, and register read/write operations.
 */

#pragma once

#include "stdint.h"
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initializes the I2C master bus on the specified pins.
 * 
 * @param sda_pin GPIO pin number for SDA.
 * @param scl_pin GPIO pin number for SCL.
 * @param bus_handle Pointer to store the created I2C bus handle.
 * @return esp_err_t ESP_OK on success, or an error code upon failure.
 */
esp_err_t ht_i2c_bus_init(int sda_pin, int scl_pin, i2c_master_bus_handle_t *bus_handle);

/**
 * @brief Adds an I2C device to an existing master bus.
 * 
 * @param bus_handle The handle of the initialized I2C bus.
 * @param dev_addr The 7-bit hardware address of the I2C device.
 * @param clk_speed Clock speed in Hz (e.g., 400000 for Fast Mode).
 * @param dev_handle Pointer to store the created device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Scans the I2C bus and prints the addresses of discovered devices to the console.
 * 
 * @param bus_handle The handle of the I2C bus to scan.
 */
void ht_i2c_scan(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Writes a buffer of data to a 16-bit register address.
 * 
 * @param dev_handle The handle of the target I2C device.
 * @param reg_addr The 16-bit register address to write to.
 * @param data Pointer to the data buffer to send.
 * @param len Number of bytes to send.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_i2c_write_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, const uint8_t *data, size_t len);

/**
 * @brief Reads a buffer of data from a 16-bit register address.
 * 
 * @param dev_handle The handle of the target I2C device.
 * @param reg_addr The 16-bit register address to read from.
 * @param data Pointer to the buffer where received data will be stored.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_i2c_read_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Writes a buffer of data to an 8-bit register address.
 * 
 * @param dev_handle The handle of the target I2C device.
 * @param reg_addr The 8-bit register address to write to.
 * @param data Pointer to the data buffer to send.
 * @param len Number of bytes to send.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_i2c_write_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, const uint8_t *data, size_t len);

/**
 * @brief Reads a buffer of data from an 8-bit register address.
 * 
 * @param dev_handle The handle of the target I2C device.
 * @param reg_addr The 8-bit register address to read from.
 * @param data Pointer to the buffer where received data will be stored.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_i2c_read_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);