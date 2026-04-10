#include "pico/stdlib.h"
#include "pindefs.h"
#include "hardware/i2c.h"
#include "is32fl3193.h"

#define IS31_ADDR 0x68

#define REG_SHDN           0x00
#define REG_LEDMODE        0x01 // NB: Switched on datasheet
#define REG_BREATHCTRL     0x02 // NB: Switched on datasheet
#define REG_CURRSET        0x03
#define REG_PWM0           0x04
#define REG_PWM1           0x05
#define REG_PWM2           0x06
#define REG_DATAUPDATE     0x07

void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[4] = {REG_PWM0, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, rgb, 4, false);
    i2c_write_blocking(I2C_PORT, IS31_ADDR, (const uint8_t[]){REG_DATAUPDATE, 0x00}, 2, false); // pwm data
}

void rgb_set_auto(bool enable, bool led)
{
    const uint8_t cmd[2] = {REG_SHDN, led ? (uint8_t)0x20 : (uint8_t)0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, cmd, 2, false);

    const uint8_t rgb_colors[3][3] = {{255, 0, 0}, {1, 20, 7}, {0, 0, 255}}; // r, g, b
    rgb_set_rgb(rgb_colors[enable][0], rgb_colors[enable][1], rgb_colors[enable][2]);
}

void rgb_set_breathing(bool breathing)
{
    const uint8_t cmd[2] = {REG_BREATHCTRL, breathing ? (uint8_t)0x20 : (uint8_t)0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, cmd, 2, false);
}

void rgb_init()
{
    gpio_put(IS31_POW_EN, 1);
    i2c_write_blocking(I2C_PORT, IS31_ADDR, (const uint8_t[2]){REG_CURRSET, 0x08}, 2, false); // Set max current to 5 mA
}

