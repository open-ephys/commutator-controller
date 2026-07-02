#include "cap1296.h"

#define CAP1296_ADDR 0x28

#define REG_SENSITIVITY     0x1F  // Sensitivity Control
#define REG_INPUTEN         0x21 // Sensor Input Enable
#define REG_CAL             0x26 // Calibration Activate and Status
#define REG_INTEN           0x27 // Interrupt Enable
#define REG_REPEAERATEEN    0x28  // Repeat Rate Enable
#define REG_CONF2           0x44  // Configuration 2
#define REG_MAINCONTROL     0x00 // Main Control
#define REG_INPUTSTATUS     0x03 // Sensor Input Status

void cap1296_init()
{
    // Set sensitivity multiplier to x4 and data scaling factor to x256
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_SENSITIVITY, 0x5F}, 2, false);

    // Monitor touch inputs CS1, CS2, CS3, CS4 and don't monitor touch inputs CS5, CS6 in the active state
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_INPUTEN, 0x0F}, 2, false);

    // Force recalibration of the touch inputs CS1, CS2, CS3, CS4
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_CAL, 0x0F}, 2, false);

    // Enable interrupts for touch inputs CS1, CS2, CS3, CS4 and disable interrupts for touch inputs CS5, CS6
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_INTEN, 0x0F}, 2, false);

    // Disable repeat interrupts during button holds for all touch inputs
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_REPEAERATEEN, 0x00}, 2, false);

    // Set interrupts to trigger when a button is released and recalibrate a touch input if its base count is out of limit
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[2]){REG_CONF2, 0x40}, 2, false);
}

void cap1296_clear_int_bit_in_main_control_register()
{
    uint8_t main_control_register_value;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[1]){REG_MAINCONTROL}, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, &main_control_register_value, 1, false);

    // Main Control Register
    // Clear the int bit while maintaining previous values of other bits
    const uint8_t main_control_register[2] = {REG_MAINCONTROL, (uint8_t)(main_control_register_value & 0b11111110)};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, main_control_register, 2, false);
}

uint8_t cap1296_read_sensor_input_status_register()
{
    uint8_t touch_status_buffer;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, (const uint8_t[1]){REG_INPUTSTATUS}, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, &touch_status_buffer, 1, false);
    return touch_status_buffer;
}
