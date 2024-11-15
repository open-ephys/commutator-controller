#pragma once

#include <iostream>
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "defs.h"
#include "pindefs.h"

#define IS31FL3193_ADDR 0x68

void rgb_init();
void rgb_set_breathing(bool breathing);
void rgb_set_rgb();
void rgb_set_blue();
void rgb_set_red();
void rgb_set_green();
void rgb_update(Context context);
