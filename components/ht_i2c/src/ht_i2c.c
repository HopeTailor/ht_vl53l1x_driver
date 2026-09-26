#include "ht_i2c.h"

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