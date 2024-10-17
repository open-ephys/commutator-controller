#include "pico/stdlib.h"
#include "pindefs.h"
#include "hardware/i2c.h"
#include "is32fl3193.h"

void rgb_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[4] = {0x04, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, rgb, 4, false);

    const uint8_t pwm_data[2]  = {0x07, 0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, pwm_data, 2, false);
}

void rgb_set_breathing(bool breathing)
{
    if (breathing)
    {
        const uint8_t buf[2] = {0x02, 0x20};
        i2c_write_blocking(I2C_PORT, IS31_ADDR, buf, 2, false);
    } else 
    {
        const uint8_t buf[2] = {0x02, 0x00};
        i2c_write_blocking(I2C_PORT, IS31_ADDR, buf, 2, false);
    }
}

void rgb_init()
{
    gpio_init(IS31_POW_EN);
    gpio_set_dir(IS31_POW_EN, GPIO_OUT);
    gpio_put(IS31_POW_EN, 1);

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Set max current to 5 mA
    const uint8_t max_curr[2] = {0x03, 0x08};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, max_curr, 2, false);

    // Enable current driver
    const uint8_t en_curr[2]  = {0x00, 0x20};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, en_curr, 2, false);
}