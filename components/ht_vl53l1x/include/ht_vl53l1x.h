/**
 * @file ht_vl53l1x.h
 * @brief Professional VL53L1X Time-of-Flight (ToF) Sensor Driver for ESP-IDF
 * 
 * Implements the ST Ultra Lite Driver (ULD) architecture for robust distance 
 * measuring, supporting custom ROI (Region of Interest), timing budgets, and 
 * inter-measurement periods.
 */

#pragma once 

#include "ht_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* I2C Address */
#define VL53L1X_ADDR 0x29

/* VL53L1X Core Register Map */
#define VL53L1X_REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND     0x0008
#define VL53L1X_REG_GPIO_HV_MUX_CTRL                         0x0030
#define VL53L1X_REG_GPIO_TIO_HV_STATUS                       0x0031
#define VL53L1X_REG_PHASECAL_CONFIG_TIMEOUT_MACROP           0x004B
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI         0x005E
#define VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_A              0x0060
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI         0x0061
#define VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_B              0x0063
#define VL53L1X_REG_RANGE_CONFIG_VALID_PHASE_HIGH            0x0069
#define VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD           0x006C
#define VL53L1X_REG_SD_CONFIG_WOI_SD0                        0x0078
#define VL53L1X_REG_SD_CONFIG_INITIAL_PHASE_SD0              0x007A
#define VL53L1X_REG_ROI_CONFIG_USER_ROI_CENTRE_SPAD          0x007F
#define VL53L1X_REG_ROI_CONFIG_USER_ROI_REQUESTED_RESOLUTION 0x0080
#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR                   0x0086
#define VL53L1X_REG_SYSTEM_MODE_START                        0x0087
#define VL53L1X_REG_RESULT_RANGE_STATUS                      0x0089
#define VL53L1X_REG_RESULT_FINAL_RANGE                       0x0096
#define VL53L1X_REG_RESULT_OSC_CALIBRATE_VAL                 0x00DE
#define VL53L1X_REG_FIRMWARE_SYSTEM_STATUS                   0x00E5

/* Distance Modes */
#define VL53L1X_DIST_SHORT 1  /**< Short range: Up to ~1.3m, more robust to ambient light */
#define VL53L1X_DIST_LONG  2  /**< Long range: Up to ~4.0m, highly sensitive to ambient light */

/**
 * @brief Core VL53L1X device configuration and state structure.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;  /**< Handle for the VL53L1X I2C connection */
    uint8_t distance_mode;            /**< Distance mode: VL53L1X_DIST_SHORT or VL53L1X_DIST_LONG */
    uint16_t timing_budget_ms;        /**< Time allocated for a single measurement (e.g., 15, 20, 33, 50, 100, 200, 500 ms) */
    uint32_t inter_measurement_ms;    /**< Delay between measurements in ms (MUST be >= timing_budget_ms) */
    uint8_t roi_width;                /**< Region of Interest width (Min: 4, Max: 16) */
    uint8_t roi_height;               /**< Region of Interest height (Min: 4, Max: 16) */
    uint8_t roi_center;               /**< SPAD array center index (Default: 199 for center alignment) */
} ht_vl53l1x_dev_t;

/**
 * @brief Boots up the VL53L1X, performs VHV calibration, and applies the user configuration.
 * 
 * @param dev Pointer to the device configuration structure.
 * @return esp_err_t ESP_OK on success, or timeout/I2C error code.
 */
esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev);

/**
 * @brief Starts continuous distance measurement based on inter_measurement_ms.
 * 
 * @param dev Pointer to the device configuration structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev);

/**
 * @brief Stops distance measurement and puts the laser in standby.
 * 
 * @param dev Pointer to the device configuration structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_stop_ranging(ht_vl53l1x_dev_t *dev);

/**
 * @brief Checks the hardware interrupt polarity to determine if a new measurement is ready.
 * 
 * @param dev Pointer to the device configuration structure.
 * @param is_ready Output pointer: 1 if new data is ready, 0 otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready);

/**
 * @brief Reads the final distance measurement from the sensor registers.
 * 
 * @param dev Pointer to the device configuration structure.
 * @param distance Output pointer to store the measured distance in millimeters.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_get_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance);

/**
 * @brief Retrieves the status of the last measurement to detect errors (e.g., target out of bounds).
 * 
 * @param dev Pointer to the device configuration structure.
 * @param status Output pointer to store the status code (0 means valid data).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_get_range_status(ht_vl53l1x_dev_t *dev, uint8_t *status);

/**
 * @brief Clears the interrupt flag. MUST be called after reading data to trigger the next measurement.
 * 
 * @param dev Pointer to the device configuration structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev);