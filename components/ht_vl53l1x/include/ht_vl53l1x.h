#pragma once 

#include "ht_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define VL53L1X_ADDR 0x29

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

#define VL53L1X_DIST_SHORT 1
#define VL53L1X_DIST_LONG  2


typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t distance_mode;
    uint16_t timing_budget_ms;
    uint32_t inter_measurement_ms;
    uint8_t roi_width;
    uint8_t roi_height;
    uint8_t roi_center;
} ht_vl53l1x_dev_t;

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_stop_ranging(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready);

esp_err_t ht_vl53l1x_get_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance);

esp_err_t ht_vl53l1x_get_range_status(ht_vl53l1x_dev_t *dev, uint8_t *status);

esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev);

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev);