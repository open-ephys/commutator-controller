#pragma once

#include "pico/stdlib.h"

// LTC4425 super cap charger
#define LTC4425_VMID_SEL        26
#define LTC4425_EN              28
#define LTC4425_nPOW_FAIL       29
#define LTC4425_CHARGE_CURR     28 // ADC 1

// IS31FL3193 RGB LED driver
#define IS31_POW_EN             0
#define IS31_POW_BW             1

// I2C
#define I2C_PORT                i2c1
#define I2C_SDA                 2
#define I2C_SCL                 3

// Stepper driver pins
#define TMC2130_DIR             18
#define TMC2130_STEP            19
#define TMC2130_CFG0_MISO       20
#define TMC2130_CFG1_MOSI       23
#define TMC2130_CFG2_SCLK       22
#define TMC2130_CFG3_CS         21
#define TMC2130_CFG6_EN         17

