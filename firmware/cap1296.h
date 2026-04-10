#pragma once

#include "hardware/i2c.h"
#include "pindefs.h"

#define BUTTON_RELEASE 0
#define CW_BUTTON_PRESS (1 << 0)
#define ENABLE_BUTTON_PRESS (1 << 1)
#define LED_BUTTON_PRESS (1 << 2)
#define CCW_BUTTON_PRESS (1 << 3)

void cap1296_init();
void cap1296_clear_int_bit_in_main_control_register();
uint8_t cap1296_read_sensor_input_status_register();