#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pindefs.h"
#include "defs.h"

#define CAP1296_ADDR 0x28
#define CAP1296_ALERT 11

void cap1296_init();
void cap1296_clear_touch_status();
uint8_t cap1296_read_touch_status();