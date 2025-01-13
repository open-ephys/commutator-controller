#pragma once

#include "hardware/adc.h"

#define RPROG                   2000.0 // Charge current programming resistor (Ohms)
#define CODE_TO_AMPS            (3.3 / 4096 * 1000.0 / RPROG) // 12-bit conversion


static inline float ltc4425_charge_current()
{
    uint16_t result = adc_read();
    return result * CODE_TO_AMPS;
}

void ltc4425_init();
static inline bool ltc4425_power_good() {return gpio_get(LTC4425_nPOW_FAIL);}
