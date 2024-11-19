#pragma once

#include "accelstepper/AccelStepper.h"

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// Controller state
struct Context {
    bool led = true;
    bool enable = true;
};

struct MotorContext 
{
    AccelStepper motor;
    double target_turns = 0.0;
};