#include "cap1296.h"

void cap1296_init()
{
    const uint8_t cap1298_touch_interrupts[2] = {0x27, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_interrupts, 2, false);
    const uint8_t cap1298_touch_enables[2] = {0x21, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_enables, 2, false);
    const uint8_t cap1298_touch_disable_autorepeat[2] = {0x28, 0x00};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_disable_autorepeat, 2, false);
    const uint8_t cap1298_adjust_sensitivity[2] = {0x1f, 0x5f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_adjust_sensitivity, 2, false);
    const uint8_t cap1298_force_calibrate[2] = {0x26, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_force_calibrate, 2, false);
    const uint8_t cap1298_config2[2] = {0x44, 0x41};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_config2, 2, false);
}

void cap1296_clear_touch_status()
{
    const uint8_t cap1296_clear_alert[2] = {0, 0};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1296_clear_alert, 2, false);
}
uint8_t cap1296_read_touch_status()
{
    const uint8_t cap1298_touch_status_register = 0x03;
    uint8_t touch_status_buffer[1];
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_touch_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, touch_status_buffer, 1, false);
    return touch_status_buffer[0];
}
