#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "pindefs.h"
#include "ltc4425.h"
#include "is32fl3193.h"
#include "tmc2130.h"

int main() 
{
    // Initialize chips
    stdio_init_all();
    ltc4425_init();
    rgb_init();
    tmc2130_init();

    // Wait for chips to wake up
    sleep_ms(10);

    // Wait for supercaps to charge
    RGB_SET_RED
    rgb_set_breathing(true);
    while(!gpio_get(LTC4425_nPOW_FAIL))
    {
        printf("Charge current: %f A\n", ltc4425_charge_current());
        sleep_ms(10);
    }
    rgb_set_breathing(false);
    RGB_SET_GREEN

    tmc2130_enable(1);

    return 0;
}
