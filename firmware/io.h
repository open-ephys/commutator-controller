#pragma once

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pindefs.h"
#include "defs.h"

extern volatile bool alert_flag;
void io_alert_irq_callback(unsigned int gpio, long unsigned int events);
void io_init();
