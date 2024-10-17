#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

// LTC4425 super cap charger
#define LTC4425_VMID_SEL        26
#define LTC4425_EN              28
#define LTC4425_nPOW_FAIL       29
#define LTC4425_CHARGE_CURR     28 // ADC 1

#define CHARGE_CURR_THRESH      0.06 // Super capacitor charge current, in Amps, that must be reached to transistion to normal operation.
#define RPROG                   2000.0 // Charge current programming resistor (Ohms)
#define CODE_TO_AMPS            (3.3 / 4096 * 1000.0 / RPROG) // 12-bit conversion

// IS31FL3193 RGB LED driver
#define IS31_POW_EN             0
#define IS31_POW_BW             1

#define IS31_ADDR               0x68

// I2C
#define I2C_PORT                i2c1
#define I2C_SDA                 2
#define I2C_SCL                 3

// Stepper driver pins
#define TMC2130_DIR             14
#define TMC2130_STEP            16
#define TMC2130_CFG0_MISO       12
#define TMC2130_CFG1_MOSI       11
#define TMC2130_CFG2_SCLK       13
#define TMC2130_CFG3_CS         10
#define TMC2130_CFG6_EN         9

// Power options

void setup_io()
{
    stdio_init_all();
    adc_init();

    gpio_init(LTC4425_EN);
    gpio_set_dir(LTC4425_EN, GPIO_OUT);

    gpio_init(LTC4425_VMID_SEL);
    gpio_set_dir(LTC4425_VMID_SEL, GPIO_OUT);

    adc_init();
    adc_select_input(1);

    gpio_init(IS31_POW_EN);
    gpio_set_dir(IS31_POW_EN, GPIO_OUT);

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    // pinMode(TMC2130_CFG6_EN, GPIO_OUT);
    // pinMode(TMC2130_DIR, GPIO_OUT);
    // pinMode(TMC2130_STEP, GPIO_OUT);
    // pinMode(TMC2130_CFG3_CS, GPIO_OUT);
    // pinMode(TMC2130_CFG1_MOSI, GPIO_OUT);
    // pinMode(TMC2130_CFG0_MISO, GPIO_OUT);
    // pinMode(TMC2130_CFG2_SCLK, GPIO_OUT);

    // pinMode(MOT_POW_EN, OUTPUT);
    // pinMode(VMID_SEL, OUTPUT);
    // pinMode(nPOW_FAIL, INPUT);

    // pinMode(IS31_SHDN, OUTPUT);
}

void intialize_board()
{   

    gpio_put(LTC4425_EN, 1);
    gpio_put(LTC4425_VMID_SEL, 1); // NB: Set 2.7V across each super cap

    gpio_put(IS31_POW_EN, 1);

}

inline float charge_current()
{
    uint16_t result = adc_read();
    return result * CODE_TO_AMPS;
}

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[4] = {0x04, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, rgb, 4, false);

    const uint8_t pwm_data[2]  = {0x07, 0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, pwm_data, 2, false);
}

void setup_rgb_driver()
{
    // Set max current to 5 mA
    const uint8_t max_curr[2] = {0x03, 0x08};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, max_curr, 2, false);

    // TODO: update_rgb()
    set_rgb_color(1, 20, 7);

    // Enable current driver
    const uint8_t en_curr[2]  = {0x00, 0x20};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, en_curr, 2, false);
}

int main() 
{
    setup_io();
    intialize_board();
    setup_rgb_driver();

    while (true) 
    {
        printf("Charge current: %f A\n", charge_current());
        sleep_ms(100);
    }

    return 0;
}
