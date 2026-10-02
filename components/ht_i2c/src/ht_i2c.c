#include "ht_i2c.h"

esp_err_t ht_i2c_bus_init(int sda_pin, int scl_pin, i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t conf = {
        .i2c_port = -1,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    return i2c_new_master_bus(&conf, bus_handle);
}

void ht_i2c_scan(i2c_master_bus_handle_t bus_handle) {
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");

    for(uint8_t i = 3; i < 0x78; i++) {
        if(i % 16 == 0) {
            printf("\n%02x:", i);

            esp_err_t err = i2c_master_probe(bus_handle, i, 100);

            if(err == ESP_OK) {
                printf(" %02x", i);
            }
            else {
                printf(" --");
            }
        }
        printf("\n\n");
    }
}

esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed, i2c_master_dev_handle_t *dev_handle) {
    i2c_device_config_t conf = {
        .device_address = dev_addr,
        .scl_speed_hz = clk_speed,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7
    };
    return i2c_master_bus_add_device(bus_handle, &conf, dev_handle);
}

esp_err_t ht_i2c_write_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, const uint8_t *data, size_t len) {
    uint8_t write_buf[2 + len];
    write_buf[0] = (uint8_t)(reg_addr >> 8);
    write_buf[1] = (uint8_t)(reg_addr & 0xFF);

    for(int i = 0; i < len; i++) {
        write_buf[i + 2] = data[i];
    }

    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
}

esp_err_t ht_i2c_read_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint8_t *data, size_t len) {
    uint8_t reg_buf[2] = { (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF) };
    
    return i2c_master_transmit_receive(dev_handle, reg_buf, 2, data, len, -1);
}

esp_err_t ht_i2c_write_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, const uint8_t *data, size_t len) {
    uint8_t write_buf[1 + len];
    write_buf[0] = reg_addr;

    for(int i = 0; i < len; i++) {
        write_buf[i + 1] = data[i];
    }

    return i2c_master_transmit(dev_handle, write_buf, len + 1, -1);
}

esp_err_t ht_i2c_read_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
}


