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

#define VL53L1X_REG_SYSTEM_MODE_START      0x0087
#define VL53L1X_REG_GPIO_TIO_HV_STATUS     0x0031
#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR 0x0086
#define VL53L1X_REG_RESULT_FINAL_RANGE     0x0096

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev, i2c_port_t port) {
    dev->port = port;

    esp_err_t err = ht_i2c_write_reg16(port, VL53L1X_ADDR,  0x002D, vl53l1x_default_configuration, sizeof(vl53l1x_default_configuration));

    if(err != ESP_OK) {
        return err;
    }

    uint8_t vhv_config = 0x01;
    err = ht_i2c_write_reg16(port, VL53L1X_ADDR, 0x002E, &vhv_config, 1);

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

esp_err_t ht_vl53l1x_det_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance) {
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






