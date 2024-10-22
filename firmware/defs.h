#pragma once

#include "accelstepper/AccelStepper.h"

// TODO: sketchy because if not big enough, ends up in program memory
// There are proper ways to do this
#define FLASH_TARGET_OFFSET (1024 * 1024)

// Controller state
struct Context {
    bool led_on = true;
    bool commutator_en = true;
};

struct MotorContext 
{
    AccelStepper motor;
    double target_turns = 0.0;
};