#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

/////////////////////////////////////////////////////////////////////////////////
// OPTIONS
/////////////////////////////////////////////////////////////////////////////////

// Firmware Version
#define FIRMWARE_VER        "1.7.0"

// 1. Uncomment to continuously dump button press data over Serial
//#define DEBUG

// 2. Select a commutator type by uncommenting one of the following
//#define COMMUTATOR_TYPE     "SPI Rev. A"
//#define GEAR_RATIO          1.77777777778

#define COMMUTATOR_TYPE     "Single Channel Coax Rev. A"
#define GEAR_RATIO          2.0

//#define COMMUTATOR_TYPE     "Dual Channel Coax Rev. A"
//#define GEAR_RATIO          3.06666666667

/////////////////////////////////////////////////////////////////////////////////
// DEBUG
/////////////////////////////////////////////////////////////////////////////////
#define DEBUG

/////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
/////////////////////////////////////////////////////////////////////////////////

//---------- PINS ----------//

// IS31FL3193 RGB LED driver pins
#define IS31FL3193_SDB  4
#define IS31FL3193_BM   5

// LTC4425 super cap charger pins
#define LTC4425_SEL     26
#define LTC4425_PROG    1 //ADC, not GPIO
#define LTC4425_EN      28
#define LTC4425_PFO     29

// CAP1296 touch sensor pin
#define CAP1296_ALERT   11

// TMC2130 stepper motor driver pins
#define TMC2130_EN      17
#define TMC2130_DIR     18
#define TMC2130_STEP    19
#define TMC2130_MISO    20
#define TMC2130_CS      21
#define TMC2130_SCLK    22
#define TMC2130_MOSI    23

// I2C pins
#define I2C_SDA         2
#define I2C_SCL         3

//---------- MISCELLANEOUS ----------//

// I2C RP2040 port & device addresses
#define I2C_PORT        i2c1
#define IS31_ADDR       0x68
#define CAP1296_ADDR    0x28

// flash constants
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// Other constants
#define RPROG           2490.0 // Charge current programming resistor (Ohms)
#define CODE_TO_AMPS    (3.3 / 4096 * 1000.0 / RPROG) // 12-bit conversion

// Controller state
static struct Context {
    uint8_t led; 
    uint8_t enable;
    uint8_t speed;
    uint8_t accel;
    uint8_t target;
} ctx;

volatile uint8_t alert_flag = 0;

void cap1298_alert_callback()
{
    alert_flag = 1;
}

void init_io()
{
    // Initialize serial port
    stdio_init_all();
    // Initialize GPIO for supercap charger
    gpio_init(LTC4425_EN);
    gpio_set_dir(LTC4425_EN, GPIO_OUT);
    gpio_init(LTC4425_SEL);
    gpio_set_dir(LTC4425_SEL, GPIO_OUT);
    gpio_init(LTC4425_PFO);
    gpio_set_dir(LTC4425_SEL, GPIO_IN);
    adc_init();
    adc_select_input(LTC4425_PROG);
    // Initialize GPIO for LED driver
    gpio_init(IS31FL3193_SDB);
    gpio_set_dir(IS31FL3193_SDB, GPIO_OUT);
    // Initialize GPIO & interrupt for cap touch sensor
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &cap1298_alert_callback);
    // Initialize GPIO as i2c for cap touch sensor and LED driver 
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // Initialize GPIO for motor driver 
    // gpio_set_dir(TMC2130_EN, GPIO_OUT);
    // gpio_set_dir(TMC2130_DIR, GPIO_OUT);
    // gpio_set_dir(TMC2130_STEP, GPIO_OUT);
    // gpio_set_dir(TMC2130_CS, GPIO_OUT);
    // gpio_set_dir(TMC2130_MOSI, GPIO_OUT);
    // gpio_set_dir(TMC2130_MISO, GPIO_OUT);
    // gpio_set_dir(TMC2130_SCLK, GPIO_OUT);
}

float read_charge_current()
{
    uint16_t result = adc_read();
    return result * CODE_TO_AMPS;
}

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t rgb[4] = {0x04, r, g, b};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, rgb, 4, false);
    const uint8_t pwm[2]  = {0x07, 0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, pwm, 2, false);
}

void update_rgb(struct Context context)
{
    if (!context.led) 
    {
        set_rgb_color(0x00, 0x00, 0x00);
    }
    else if (!context.enable) 
    {
        set_rgb_color(255, 0, 0);
    }
    else
    {
        set_rgb_color(1, 20, 7);
    }
    return;
}

void init_rgb()
{
    gpio_put(IS31FL3193_SDB, 1);
    const uint8_t max_curr[2] = {0x03, 0x08}; // Set max current to 5 mA
    i2c_write_blocking(I2C_PORT, IS31_ADDR, max_curr, 2, false);
    const uint8_t breathe_mode_on[2] = {0x02, 0x20}; // Set the LED to breathe during setup
    i2c_write_blocking(I2C_PORT, IS31_ADDR, breathe_mode_on, 2, false);
    const uint8_t en_curr[2]  = {0x00, 0x20}; // Enable current driver
    i2c_write_blocking(I2C_PORT, IS31_ADDR, en_curr, 2, false);
}

void init_power() // Stabilize charge current and breath LED in meantime
{
    gpio_put(LTC4425_SEL, 1); // Set 2.7V across each super cap
    gpio_put(LTC4425_EN, 1);
    set_rgb_color(255, 0, 0);
    while (gpio_get(LTC4425_PFO) == 0){}
    const uint8_t breathe_mode_off[2] = {0x02, 0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, breathe_mode_off, 2, false);
}

void init_touch()
{
    const uint8_t cap1298_touch_interrupts[2] = {0x27, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_interrupts, 2, false);
    const uint8_t cap1298_touch_enables[2] = {0x21, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_enables, 2, false);
    const uint8_t cap1298_touch_disable_autorepeat[2] = {0x28, 0x00};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_touch_disable_autorepeat, 2, false);
    const uint8_t cap1298_adjust_sensitivity[2] = {0x1f, 0x5f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_adjust_sensitivity, 2, false);
    const uint8_t cap1298_force_calibrate[2] = {0x26, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_force_calibrate, 2, false);
    const uint8_t cap1298_config2[2] = {0x44, 0x41};
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_config2, 2, false);
}

void print_diagnostics()
{
    uint8_t *buffer;
    sleep_ms(15);
    const uint8_t cap1298_device_id_register = 0xfd;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_device_id_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    sleep_ms(500);
    printf("device id: %02hhx\n", buffer[0]);
    const uint8_t cap1298_control_register = 0x00;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_control_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("control: %02hhx\n", buffer[0]);
    const uint8_t cap1298_status_register = 0x02;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("status: %02hhx\n", buffer[0]);
    const uint8_t cap1298_calibration_status_register = 0x26;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_calibration_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("calibration: %02hhx\n", buffer[0]);
    const uint8_t cap1298_noise_register = 0x0a;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_noise_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("noise: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal1_register = 0xb1;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_cal1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal1: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal2_register = 0xb2;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_cal2_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal2: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal3_register = 0xb3;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_cal3_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal3: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal4_register = 0xb4;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_cal4_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal4: %02hhx\n", buffer[0]);
    const uint8_t cap1298_lsb1_register = 0xb9;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_lsb1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("lsb1: %02hhx\n", buffer[0]);
}

void turn_motor(uint8_t direction, struct Context context)
{
    if (direction){printf("cw\n");}
    if (!direction){printf("ccw\n");}
}

// int find_first_empty_page()
// {
//     uint8_t page, *addr;
//     for (page = 0; page < FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE; page++)
//     {
//         *addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
//         for (uint8_t i = 0; i < sizeof(struct Context); i++)
//         {
//             if (*(addr + i) != 0xff)
//             {
//                 return page;
//             }
//         }
//     }
// }

void write_flash_context()
{
    // int first_empty_page = find_first_empty_page();    
    // uint8_t *context_as_bytes = (uint8_t *) &context;
    // const uint32_t interrupts = save_and_disable_interrupts();
    // if (first_empty_page < 0)
    // {
    //     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    //     first_empty_page = 0;
    // }
    // flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (const uint8_t *)context_as_bytes, FLASH_PAGE_SIZE);
    // restore_interrupts(interrupts);

    // these two flash functions are unsafe. we must disable interrupts.
    uint8_t *context_as_bytes = (uint8_t *) &ctx;
    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)context_as_bytes, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
}

void read_flash_context()
{
    // int first_empty_page = find_first_empty_page();
    // return *(struct Context *)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&ctx, (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET), sizeof(struct Context));
}

int main() 
{
    // struct Context ctx;
    ctx.enable = 0;
    uint8_t touch_status_buffer[1];
    uint8_t current_touch_data, previous_touch_data;
    const uint8_t cap1298_clear_alert[2] = {0x00, 0x00};
    const uint8_t cap1298_touch_status_register = 0x03;
    uint8_t turn_flag;
    init_io();
    init_rgb();
    init_power();
    init_touch();
    #ifdef DEBUG
    uint8_t button_counter;
    print_diagnostics();
    #endif
    // ctx = read_flash_context();
    read_flash_context();
    update_rgb(ctx);
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_clear_alert, 2, false); // Clear touch status register after initialization
    while (true) 
    {   
        if (alert_flag == 1)
        {
            alert_flag = 0;
            i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1298_touch_status_register, 1, true); // Read from touch status register
            i2c_read_blocking(I2C_PORT, CAP1296_ADDR, touch_status_buffer, 1, false); // Copy value of touch status register to buffer
            i2c_write_blocking(I2C_PORT, CAP1296_ADDR, cap1298_clear_alert, 2, false); // Clear touch status register
            // This logic is required to deal with the fact that the release from the previously touched button is not cleared 
            if (touch_status_buffer[0] & (touch_status_buffer[0] - 1)) 
            { // If more than one bit is set, mask out the bit that corresponds to the previously button
                current_touch_data = (previous_touch_data ^ touch_status_buffer[0]) & touch_status_buffer[0];
            }
            else
            { // If one bit is set, then the current button press is the same as the previous button press
                current_touch_data = touch_status_buffer[0];
            }
            previous_touch_data = current_touch_data;
            #ifdef DEBUG
            printf("state: %02hhx, button: %02hhx, count: %u\n", touch_status_buffer[0], current_touch_data, ++button_counter);
            #endif
            switch (current_touch_data)
            {
                case 0x04: // top button
                    ctx.led = !ctx.led;
                    write_flash_context();
                    update_rgb(ctx);
                    break;
                case 0x02: // bottom button
                    ctx.enable = !ctx.enable;
                    write_flash_context();
                    update_rgb(ctx);
                    break;
                case 0x01: // left button
                    turn_motor(0, ctx);
                    break;
                case 0x08: // right button
                    turn_motor(1, ctx);
                    break;
            }
        }
    }
    return 0;
}
