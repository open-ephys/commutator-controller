#include "is32fl3193.h"

void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t rgb[4] = {0x04, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, rgb, 4, false);
    const uint8_t pwm_data[2] = {0x07, 0x00};
    i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, pwm_data, 2, false);
}

void rgb_set_blue() { rgb_set_rgb(0, 0, 255); }
void rgb_set_red() { rgb_set_rgb(255, 0, 0); }
void rgb_set_green() { rgb_set_rgb(1, 20, 7); }

void rgb_update(Context context)
{
    if (context.led)
    { // Enable current driver
        const uint8_t en_curr[2] = {0x00, 0x20};
        i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, en_curr, 2, false);
    }
    else
    { // Disable current driver
        const uint8_t en_curr[2] = {0x00, 0x00};
        i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, en_curr, 2, false);
    }

    if (context.enable)
    {
        rgb_set_green();
    }
    else
    {
        rgb_set_red();
    }
}

void rgb_set_breathing(bool breathing)
{
    if (breathing)
    {
        const uint8_t buf[2] = {0x02, 0x20};
        i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, buf, 2, false);
    }
    else
    {
        const uint8_t buf[2] = {0x02, 0x00};
        i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, buf, 2, false);
    }
}

void rgb_init()
{
    gpio_put(IS31FL3193_SDB, 1);
    const uint8_t max_curr[2] = {0x03, 0x08}; // Set max current to 5 mA
    i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, max_curr, 2, false);
    const uint8_t breathe_mode_on[2] = {0x02, 0x20}; // Set the LED to breathe during setup
    i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, breathe_mode_on, 2, false);
    const uint8_t en_curr[2] = {0x00, 0x20}; // Enable current driver
    i2c_write_blocking(I2C_PORT, IS31FL3193_ADDR, en_curr, 2, false);
}