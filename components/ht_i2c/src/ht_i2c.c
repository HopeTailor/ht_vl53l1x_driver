#include "ht_i2c.h"
#include "freertos/FreeRTOS.h"

#define I2C_TIMEOUT_MS 1000

esp_err_t ht_i2c_init(i2c_port_t port, int sda_pin, int scl_pin, uint32_t clk_speed) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,  
    };

    esp_err_t err = i2c_param_config(port, &conf);
    if(err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}

esp_err_t ht_i2c_write_reg16(i2c_port_t port, uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, (uint8_t)(reg_addr >> 8), true);
    i2c_master_write_byte(cmd, (uint8_t)(reg_addr & 0xff), true);

    i2c_master_write(cmd, data, len, true);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t ht_i2c_read_reg16(i2c_port_t port, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, (uint8_t)(reg_addr >> 8), true);
    i2c_master_write_byte(cmd, (uint8_t)(reg_addr & 0xff), true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if(len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }

    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return err;
}










