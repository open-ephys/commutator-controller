#pragma once

#include "hardware/adc.h"

#define RPROG                   2490.0 // Charge current programming resistor (Ohms)
#define CODE_TO_AMPS            (3300.0 / 4096.0 / RPROG) // 12-bit conversion


static inline float ltc4425_charge_current()
{
    uint16_t result = adc_read();
    return (float)result * CODE_TO_AMPS;
}

void ltc4425_init();
static inline bool ltc4425_power_good() {return gpio_get(LTC4425_nPOW_FAIL);}
