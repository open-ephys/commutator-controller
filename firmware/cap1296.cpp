#include "cap1296.h"

void cap1296_init()
{
    // Set sensitivity multiplier to x4 and data scaling factor to x256 
    const uint8_t sensitivity_control_register[2] = {0x1F, 0x5f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, sensitivity_control_register, 2, false);
    // Do monitor touch inputs CS1, CS2, CS3, CS4 and don't monitor touch inputs CS5, CS6 in the active state
    const uint8_t sensor_input_enable_register[2] = {0x21, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, sensor_input_enable_register, 2, false);
    // Force recalibration of the touch inputs CS1, CS2, CS3, CS4 
    const uint8_t calibration_activate_status_register[2] = {0x26, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, calibration_activate_status_register, 2, false);
    // Enable interrupts for touch inputs CS1, CS2, CS3, CS4 and disable interrupts for touch inputs CS5, CS6
    const uint8_t interrupt_enable_register[2] = {0x27, 0x0f}; 
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, interrupt_enable_register, 2, false);
    // Disable repeat interrupts during button holds for all touch inputs
    const uint8_t repeat_rate_enable_register[2] = {0x28, 0x00};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, repeat_rate_enable_register, 2, false);
    // Set interrupts to trigger when a button is released and recalibrate a touch input if its base
    // count is out of limit
    const uint8_t configuration_2_register[2] = {0x44, 0x40};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, configuration_2_register, 2, false);
}

void cap1296_clear_int_bit_in_main_control_register()
{
    const uint8_t main_control_register_address = 0x00;
    const uint8_t int_bit_negative_mask = 0b11111110;
    uint8_t main_control_register_value;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &main_control_register_address, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, &main_control_register_value, 1, false);
    const uint8_t main_control_register[2] = {main_control_register_address, main_control_register_value & int_bit_negative_mask};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, main_control_register, 2, false);
}

uint8_t cap1296_read_sensor_input_status_register()
{
    const uint8_t sensor_input_status_register_address = 0x03;
    uint8_t touch_status_buffer;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &sensor_input_status_register_address, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, &touch_status_buffer, 1, false);
    return touch_status_buffer;
}
