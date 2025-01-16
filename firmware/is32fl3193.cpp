#include "pico/stdlib.h"
#include "pindefs.h"
#include "hardware/i2c.h"
#include "is32fl3193.h"

void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[4] = {0x04, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, rgb, 4, false);
    i2c_write_blocking(I2C_PORT, IS31_ADDR, (const uint8_t[]){0x07, 0x00}, 2, false); // pwm data
}

void rgb_set_auto(bool enable, bool led)
{
    const uint8_t current_driver_settings[2] = {0x00, 0x20}; // {disable current driver, enable current driver}
    const uint8_t current_driver_address_value[2] = {0x00, current_driver_settings[led]}; // {enable current register address, enable current register value}
    i2c_write_blocking(I2C_PORT, IS31_ADDR, current_driver_address_value, 2, false);

    const uint8_t rgb_colors[3][3] = {{255, 0, 0}, {1, 20, 7}, {0, 0, 255}}; // r, g, b
    rgb_set_rgb(rgb_colors[enable][0], rgb_colors[enable][1], rgb_colors[enable][2]);
}

void rgb_set_breathing(bool breathing)
{
    const uint8_t breathing_settings[2] = {0x00, 0x20}; // {no breathing, yes breathing}
    const uint8_t breathing_address_value[2] = {0x02, breathing_settings[breathing]}; // {breathing register address, breathing register value}
    i2c_write_blocking(I2C_PORT, IS31_ADDR, breathing_address_value, 2, false);
}

void rgb_init()
{
    gpio_put(IS31_POW_EN, 1);
    i2c_write_blocking(I2C_PORT, IS31_ADDR, (const uint8_t[2]){0x03, 0x08}, 2, false); // Set max current to 5 mA
}

