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

// I2C
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

    i2c_init(i2c1, 400000);
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

static inline float charge_current()
{
    uint16_t result = adc_read();
    return result * CODE_TO_AMPS;
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() 
{
    
    setup_io();
    intialize_board();

    while(true){
        printf("\nI2C Bus Scan\n");
        printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

        for (int addr = 0; addr < (1 << 7); ++addr) {
            if (addr % 16 == 0) {
                printf("%02x ", addr);
            }

            // Perform a 1-byte dummy read from the probe address. If a slave
            // acknowledges this address, the function returns the number of bytes
            // transferred. If the address byte is ignored, the function returns
            // -1.

            // Skip over any reserved addresses.
            int ret;
            uint8_t rxdata;
            if (reserved_addr(addr))
                ret = PICO_ERROR_GENERIC;
            else
                ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);

            printf(ret < 0 ? "." : "@");
            printf(addr % 16 == 15 ? "\n" : "  ");
        }
        printf("Done.\n");
    }


    intialize_board();

    while (true) 
    {
        printf("Charge current: %f A\n", charge_current());
        sleep_ms(100);
    }

    return 0;
}
