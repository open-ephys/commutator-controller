#include "pico/stdlib.h"
#include "pindefs.h"

#include "ltc4425.h"

//extern uint16_t adc_read();

void ltc4425_init()
{   
    gpio_init(LTC4425_EN);
    gpio_set_dir(LTC4425_EN, GPIO_OUT);
    gpio_put(LTC4425_EN, 1);

    gpio_init(LTC4425_VMID_SEL);
    gpio_set_dir(LTC4425_VMID_SEL, GPIO_OUT);
    gpio_put(LTC4425_VMID_SEL, 1); // NB: Set 2.7V across each super cap

    gpio_init(LTC4425_nPOW_FAIL);
    gpio_set_dir(LTC4425_nPOW_FAIL, GPIO_IN);

    adc_init();
    adc_select_input(1);
}