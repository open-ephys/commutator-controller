#pragma once

#include "pico/stdlib.h"

// LTC4425 super capacitor charger pins
#define LTC4425_VMID_SEL 26
#define LTC4425_EN 28
#define LTC4425_nPOW_FAIL 29
#define LTC4425_CHARGE_CURR 1 // ADC, not GPIO

// IS31FL3193 RGB LED driver pins
#define IS31FL3193_SDB 4
#define IS31FL3193_BM 5

// I2C port & pins for LTC4425 & IS31FL3193
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3

// CAP1296 touch sensor pin
#define CAP1296_ALERT 11

// Stepper motor driver pins
#define TMC2130_DIR 18
#define TMC2130_STEP 19
#define TMC2130_CFG0_MISO 20
#define TMC2130_CFG1_MOSI 23
#define TMC2130_CFG2_SCLK 22
#define TMC2130_CFG3_CS 21
#define TMC2130_CFG6_EN 17

// SPI port for TMC2130
#define SPI_PORT spi0