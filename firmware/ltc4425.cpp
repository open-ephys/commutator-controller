#include "ltc4425.h"

// extern uint16_t adc_read();

void ltc4425_init()
{
    gpio_put(LTC4425_EN, 1);
    gpio_put(LTC4425_VMID_SEL, 1); // NB: Set 2.7V across each super cap
}