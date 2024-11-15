#pragma once

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pindefs.h"

// TMC2130 registers
#define REG_GCONF 0x00
#define REG_GSTAT 0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF 0x6C
#define REG_COOLCONF 0x6D
#define REG_DCCTRL 0x6E
#define REG_DRVSTATUS 0x6F
#define REG_PWMCONF 0x70

static inline void tmc2130_enable(bool enable) { gpio_put(TMC2130_CFG6_EN, enable); }

void tmc2130_init();
