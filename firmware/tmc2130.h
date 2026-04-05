#pragma once

#include "pindefs.h"

#define TMC2130_SPI_PORT        spi0

// TMC2130 registers
#define REG_GCONF               0x00
#define REG_GSTAT               0x01
#define REG_IHOLD_IRUN          0x10
#define REG_TPOWERDOWN          0x11
#define REG_TSTEP               0x12
#define REG_TPWMTHRS            0x13
#define REG_TCOOLTHRS           0x14
#define REG_CHOPCONF            0x6C
#define REG_COOLCONF            0x6D
#define REG_DCCTRL              0x6E
#define REG_DRVSTATUS           0x6F
#define REG_PWMCONF             0x70

static inline void tmc2130_enable(bool enable)
{
    gpio_put(TMC2130_CFG6_EN, enable);
}

void tmc2130_init();
uint32_t tmc2130_status();
