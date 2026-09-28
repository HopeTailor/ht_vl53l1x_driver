#include "ht_vl53l1x.h"

const uint8_t vl53l1x_default_configuration[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 
    0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 
    0x00, 0x01, 0x09, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 
    0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define VL53L1X_REG_SYSTEM_MODE_START                0x0087
#define VL53L1X_REG_GPIO_TIO_HV_STATUS               0x0031
#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR           0x0086
#define VL53L1X_REG_RESULT_FINAL_RANGE               0x0096
#define VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD   0x006C
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI 0x005E
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI 0x0061

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev) {
   
    esp_err_t err = ht_i2c_write_reg16(dev->port, VL53L1X_ADDR,  0x002D, vl53l1x_default_configuration, sizeof(vl53l1x_default_configuration));

    if(err != ESP_OK) {
        return err;
    }

    uint8_t vhv_config = 0x01;
    err = ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, 0x002E, &vhv_config, 1);
    if(err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_set_timing_budget(dev);
    if(err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_set_inter_measurement(dev);
    return err;
}

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev) {
    uint8_t start_cmd = 0x40;
    return ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_SYSTEM_MODE_START, &start_cmd, 1);
}

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready) {
    uint8_t status = 0;
    esp_err_t err = ht_i2c_read_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_GPIO_TIO_HV_STATUS, &status, 1);
    if(err != ESP_OK) {
        return err;
    }

    status = status & 0x01;
    if(status == 0) {
        *is_ready = 1;
    }
    else {
        *is_ready = 0;
    }

    return ESP_OK;
}

esp_err_t ht_vl53l1x_get_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance) {
    uint8_t data[2] = {0, 0};

    esp_err_t err = ht_i2c_read_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_RESULT_FINAL_RANGE, data, 2);
    if(err != ESP_OK) {
        return err;
    }

    *distance = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev) {
    uint8_t clear_cmd = 0x01;
    return ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, &clear_cmd, 1);
}

static esp_err_t ht_vl53l1x_set_timing_budget(ht_vl53l1x_dev_t *dev) {
    if(dev->timing_budget_ms < 20) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t budget_us = dev->timing_budget_ms * 1000;

    uint32_t active_budget_us = budget_us - 4300;

    uint32_t phase_a_us = active_budget_us / 2;
    uint32_t phase_b_us = active_budget_us - phase_a_us;

    uint32_t macro_a = (phase_a_us * 1000) / 2070;
    uint32_t macro_b = (phase_b_us * 1000) / 2070;

    uint16_t a_hi = ht_vl53l1x_encode_timeout(macro_a);
    uint16_t b_hi = ht_vl53l1x_encode_timeout(macro_b);

    uint8_t data_a[2] = { (uint8_t)(a_hi >> 8), (uint8_t)(a_hi & 0xFF)};
    esp_err_t err = ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, data_a, 2);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t data_b[2] = { (uint8_t)(b_hi >> 8), (uint8_t)(b_hi & 0xFF)};
    esp_err_t err = ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, data_b, 2);
    return err;
}

static uint16_t ht_vl53l1x_encode_timeout(uint32_t timeout_macro_clks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if(timeout_macro_clks > 0) {
        ls_byte = timeout_macro_clks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte = ls_byte >> 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (uint16_t)(ls_byte & 0xFF);
    }
    return 0;
}

static esp_err_t ht_vl53l1x_set_inter_measurement(ht_vl53l1x_dev_t *dev) {
    if(dev->inter_measurement_ms < dev->timing_budget_ms) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t clock_pll = dev->inter_measurement_ms * 1000;
    uint8_t data[4];
    data[0] = (uint8_t)((clock_pll >> 24) & 0xFF);
    data[1] = (uint8_t)((clock_pll >> 16) & 0xFF);
    data[2] = (uint8_t)((clock_pll >> 8) & 0xFF);
    data[3] = (uint8_t)(clock_pll & 0xFF);

    return ht_i2c_write_reg16(dev->port, VL53L1X_ADDR, VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, data, 4);
}




