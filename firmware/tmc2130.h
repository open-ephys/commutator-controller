#pragma once

#include "pindefs.h"

// We used 64 microsteps per step
#define USTEPS_PER_STEP         64

static inline void tmc2130_enable(bool enable)
{
    gpio_put(TMC2130_CFG6_EN, enable);
}

void tmc2130_init();
uint32_t tmc2130_status();
