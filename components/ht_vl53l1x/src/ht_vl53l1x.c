#include "ht_vl53l1x.h"

static const uint8_t vl53l1x_default_configuration[91] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
};

static esp_err_t wr8(ht_vl53l1x_dev_t *dev, uint16_t reg, uint8_t v) {
    return ht_i2c_write_reg8(dev->i2c_dev, reg, &v, 1);
}

static esp_err_t wr16(ht_vl53l1x_dev_t *dev, uint16_t reg, uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v >> 8) , (uint8_t)(v & 0xFF)};
    return ht_i2c_write_reg16(dev->i2c_dev, reg, b, 2);
}

static esp_err_t ht_vl53l1x_set_distance_mode(ht_vl53l1x_dev_t *dev) {
    esp_err_t err;

    if(dev->distance_mode == VL53L1X_DIST_SHORT) {
        err = wr8(dev, VL53L1X_REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        err |= wr16(dev, VL53L1X_REG_SD_CONFIG_WOI_SD0, 0x0705);
        err |= wr16(dev, VL53L1X_REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
    }
    else if(dev->distance_mode == VL53L1X_DIST_LONG) {
        err = wr8(dev, VL53L1X_REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
        err |= wr8(dev, VL53L1X_REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
        err |= wr16(dev, VL53L1X_REG_SD_CONFIG_WOI_SD0, 0x0F0D);
        err |= wr16(dev, VL53L1X_REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }
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

static esp_err_t ht_vl53l1x_set_timing_budget(ht_vl53l1x_dev_t *dev) {
    uint16_t a = 0, b = 0;

    if(dev->distance_mode == VL53L1X_DIST_SHORT) {
        switch(dev->timing_budget_ms) {
            case 15:
                a = 0x001D;
                b = 0x0027;
                break;
            case 20:
                a = 0x0051;
                b = 0x006E;
                break;
            case 33:
                a = 0x00D6;
                b = 0x006E;
                break;
            case 50:
                a = 0x01AE;
                b = 0x01E8;
                break;
            case 100:
                a = 0x02E1;
                b = 0x0388;
                break;
            case 200:
                a = 0x03E1;
                b = 0x0496;
                break;
            case 500:
                a = 0x0591;
                b = 0x05C1;
                break;
            default:
                return ESP_ERR_INVALID_ARG;  
        }
    }
    else if(dev->distance_mode == VL53L1X_DIST_LONG) {
        switch(dev->timing_budget_ms) {
            case 20:
                a = 0x001E;
                b = 0x0022;
                break;
            case 33:
                a = 0x0060;
                b = 0x006E;
                break;
            case 50:
                a = 0x00AD;
                b = 0x00C6;
                break;
            case 100:
                a = 0x01CC;
                b = 0x01EA;
                break;
            case 200:
                a = 0x02D9;
                b = 0x02F8;
                break;
            case 500:
                a = 0x048F;
                b = 0x04A4;
                break;
            default:
                return ESP_ERR_INVALID_ARG;   
        }
    }
    else {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = wr16(dev, VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, a);
    err |= wr16(dev, VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, b);
    return err;
}

static esp_err_t ht_vl53l1x_set_inter_measurement(ht_vl53l1x_dev_t *dev) {
    if(dev->inter_measurement_ms < dev->timing_budget_ms) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t osc[2] = {0, 0};
    esp_err_t err = ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_RESULT_OSC_CALIBRATE_VAL, osc, 2);
    if(err != ESP_OK) {
        return err;
    }
    uint16_t clock_pll = (uint16_t)(((osc[0] << 8) | osc[1]) & 0x3FF);
    uint32_t period = (uint32_t)(clock_pll * dev->inter_measurement_ms * 1.075f);

    uint8_t data[4] = {(uint8_t)(period >> 24), (uint8_t)(period >> 16)< (uint8_t)(period >> 8), (uint8_t)(period & 0xFF)};
    return ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, data, 4);
}

static esp_err_t ht_vl53l1x_set_roi(ht_vl53l1x_dev_t *dev) {
    if(dev->roi_height < 4 || dev->roi_width < 4 || dev->roi_height > 16 || dev->roi_width > 16) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t center = dev->roi_center;
    if(dev->roi_width > 10 || dev->roi_height > 10) {
        center = 199;
    }

    uint8_t size = (uint8_t)(((dev->roi_height - 1) << 4) | (dev->roi_width - 1));
    esp_err_t err = wr8(dev, VL53L1X_REG_ROI_CONFIG_USER_ROI_CENTRE_SPAD, center);
    err |= wr8(dev, VL53L1X_REG_ROI_CONFIG_USER_ROI_REQUESTED_RESOLUTION, size);
    return err;
}

esp_err_t ht_vl53l1x_init(ht_vl53l1x_dev_t *dev) {
    esp_err_t err = ESP_OK;

    uint8_t booted = 0;
    int tries = 0;
    while(booted == 0) {
        if(ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, &booted, 1) != ESP_OK) {
            booted = 0;
        }
        if(++tries > 200) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for(uint16_t i = 0; i < sizeof(vl53l1x_default_configuration); i++) {
        err |= wr8(dev, 0x002D + i, vl53l1x_default_configuration[i]);
    }
    if(err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_stop_ranging(dev);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t ready = 0;
    for(int t = 0; t < 200 && !ready; t++) {
        if(ht_vl53l1x_check_data_ready(dev, &ready) != ESP_OK) {
            ready = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if(!ready) {
        return ESP_ERR_TIMEOUT;
    }

    err = ht_vl53l1x_clear_interrupt(dev);
    if(err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_stop_ranging(dev);
    if(err != ESP_OK) {
        return err;
    }

    err  = wr8(dev, VL53L1X_REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    err |= wr8(dev, 0x000B, 0x00);                                   
    if (err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_set_distance_mode(dev);
    if (err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_set_timing_budget(dev);
    if (err != ESP_OK) {
        return err;
    }

    err = ht_vl53l1x_set_inter_measurement(dev);
    if (err != ESP_OK) {
        return err;
    }

    return ht_vl53l1x_set_roi(dev);
}

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev) {
    return wr8(dev, VL53L1X_REG_SYSTEM_MODE_START, 0x40);
}

esp_err_t ht_vl53l1x_stop_ranging(ht_vl53l1x_dev_t *dev) {
    return wr8(dev, VL53L1X_REG_SYSTEM_MODE_START, 0x00);
}

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, uint8_t *is_ready) {
    uint8_t mux = 0, st = 0;
    esp_err_t err = ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_GPIO_HV_MUX_CTRL, &mux, 1);
    if(err != ESP_OK) {
        return err;
    }
    err = ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_GPIO_TIO_HV_STATUS, &st, 1);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t int_pol;
    if(mux & 0x10) {
        int_pol = 0;
    } 
    else {
        int_pol = 1;
    }

    if((st & 0x01) == int_pol) {
        *is_ready = 1;
    } 
    else {
        *is_ready = 0;
    }
    return ESP_OK;
}

esp_err_t ht_vl53l1x_get_distance(ht_vl53l1x_dev_t *dev, uint16_t *distance) {
    uint8_t data[2] = {0, 0};

    esp_err_t err = ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_RESULT_FINAL_RANGE, data, 2);
    if(err != ESP_OK) {
        return err;
    }

    *distance = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t ht_vl53l1x_clear_interrupt(ht_vl53l1x_dev_t *dev) {
    return wr8(dev, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

esp_err_t ht_vl53l1x_get_range_status(ht_vl53l1x_dev_t *dev, uint8_t *status) {
    uint8_t raw = 0;
    esp_err_t err = ht_i2c_read_reg16(dev->i2c_dev, VL53L1X_REG_RESULT_RANGE_STATUS, &raw, 1);
    if (err != ESP_OK) return err;
    switch (raw & 0x1F) {
        case 9:  
            *status = 0;  
            break;
        case 6:  
            *status = 1;  
            break;
        case 4:  
            *status = 2;  
            break;
        case 8:  
            *status = 3;  
            break;
        case 5:  
            *status = 4;  
            break;
        case 3:  
            *status = 5;  
            break;
        case 19: 
            *status = 6;  
            break;
        case 7:  
            *status = 7;  
            break;
        case 12: 
            *status = 9;  
            break;
        case 18: 
            *status = 10; 
            break;
        case 22: 
            *status = 11; 
            break;
        case 23: 
            *status = 12; 
            break;
        case 13: 
            *status = 13; 
            break;
        default: 
            *status = 255; 
            break;
    }
    return ESP_OK;
}


