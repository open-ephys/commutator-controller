#pragma once

#include "AccelStepper.h"

// TODO: sketchy because if not big enough, ends up in program memory
// There are proper ways to do this
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

extern struct Context
{
    uint8_t led;
    uint8_t enable;
    uint8_t speed;
    uint8_t accel;
    uint8_t target;
} ctx;

extern struct MotorContext
{
    AccelStepper motor;
    double target_turns = 0.0;
} motor_ctx;