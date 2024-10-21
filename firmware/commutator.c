#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

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
// CONSTANTS
/////////////////////////////////////////////////////////////////////////////////

// IS31FL3193 RGB LED driver pins
#define IS31_SDB                0
#define IS31_BM                 1

// LTC4425 super cap charger pins
#define LTC44_SEL               26
#define LTC44_PROG              1 //ADC, not GPIO
#define LTC44_EN                28
#define LTC44_PFO               29

// CAP1296 touch sensor pin
#define CAP12_ALERT             11

// I2C pins
#define I2C_SDA                 2
#define I2C_SCL                 3

// TMC2130 stepper motor driver pins
#define TMC2130_EN              17
#define TMC2130_DIR             18
#define TMC2130_STEP            19
#define TMC2130_MISO            20
#define TMC2130_CS              21
#define TMC2130_SCLK            22
#define TMC2130_MOSI            23

// I2C RP2040 port & device addresses/registers
#define I2C_PORT                i2c1
#define IS31_ADDR               0x68
#define CAP12_ADDR              0x28

// miscellaneous constants
#define RPROG                   2490.0 // Charge current programming resistor (Ohms)
#define CODE_TO_AMPS            (3.3 / 4096 * 1000.0 / RPROG) // 12-bit conversion

// Power options

volatile uint8_t alert_flag = 0;

// Controller state
struct Context {
    uint8_t led; 
    uint8_t enable;
    uint8_t speed;
    uint8_t accel;
    uint8_t target;
};

void cap1298_alert_callback()
{
    alert_flag = 1;
}

void setup_io()
{
    stdio_init_all();
    adc_init();

    gpio_init(LTC44_EN);
    gpio_set_dir(LTC44_EN, GPIO_OUT);
    gpio_init(LTC44_SEL);
    gpio_set_dir(LTC44_SEL, GPIO_OUT);
    gpio_init(LTC44_PFO);
    gpio_set_dir(LTC44_SEL, GPIO_IN);
    adc_init();
    adc_select_input(LTC44_PROG);

    gpio_init(IS31_SDB);
    gpio_set_dir(IS31_SDB, GPIO_OUT);

    // gpio_init(CAP12_ALERT);
    // gpio_set_dir(CAP12_ALERT, GPIO_IN);
    gpio_set_irq_enabled_with_callback(CAP12_ALERT, GPIO_IRQ_EDGE_FALL, true, &cap1298_alert_callback);

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);


    // gpio_set_dir(TMC2130_EN, GPIO_OUT);
    // gpio_set_dir(TMC2130_DIR, GPIO_OUT);
    // gpio_set_dir(TMC2130_STEP, GPIO_OUT);
    // gpio_set_dir(TMC2130_CS, GPIO_OUT);
    // gpio_set_dir(TMC2130_MOSI, GPIO_OUT);
    // gpio_set_dir(TMC2130_MISO, GPIO_OUT);
    // gpio_set_dir(TMC2130_SCLK, GPIO_OUT);
}

float charge_current()
{
    uint16_t result = adc_read();
    return result * CODE_TO_AMPS;
}

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[4] = {0x04, r, g, b};
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

void setup_rgb()
{
    gpio_put(IS31_SDB, 1);
    const uint8_t max_curr[2] = {0x03, 0x08}; // Set max current to 5 mA
    i2c_write_blocking(I2C_PORT, IS31_ADDR, max_curr, 2, false);
    const uint8_t en_curr[2]  = {0x00, 0x20}; // Enable current driver
    i2c_write_blocking(I2C_PORT, IS31_ADDR, en_curr, 2, false);
}

void setup_power() // Stabilize charge current and breath LED in meantime
{
    gpio_put(LTC44_SEL, 1); // NB: Set 2.7V across each super cap
    gpio_put(LTC44_EN, 1);
    set_rgb_color(255, 0, 0);
    const uint8_t breathe_mode_on[2] = {0x02, 0x20};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, breathe_mode_on, 2, false);
    while (gpio_get(LTC44_PFO) == 0){}
    const uint8_t breathe_mode_off[2] = {0x02, 0x00};
    i2c_write_blocking(I2C_PORT, IS31_ADDR, breathe_mode_off, 2, false);
}

void setup_touch()
{
    const uint8_t cap1298_button_interrupts[2] = {0x27, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_button_interrupts, 2, false);
    const uint8_t cap1298_button_enables[2] = {0x21, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_button_enables, 2, false);
    const uint8_t cap1298_button_autorepeat[2] = {0x28, 0x00};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_button_autorepeat, 2, false);
    const uint8_t cap1298_sensitivity[2] = {0x1f, 0x5f};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_sensitivity, 2, false);
    const uint8_t cap1298_calibrate[2] = {0x26, 0x0f};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_calibrate, 2, false);
    const uint8_t cap1298_config2[2] = {0x44, 0x41};
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_config2, 2, false);
}

void print_diagnostics()
{
    uint8_t buffer[1] = {0};
    sleep_ms(15);
    const uint8_t cap1298_device_id_register = 0xfd;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_device_id_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    sleep_ms(500);
    printf("device id: %02hhx\n", buffer[0]);
    const uint8_t cap1298_control_register = 0x00;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_control_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("control: %02hhx\n", buffer[0]);
    const uint8_t cap1298_status_register = 0x02;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("status: %02hhx\n", buffer[0]);
    const uint8_t cap1298_calibration_status_register = 0x26;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_calibration_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("calibration: %02hhx\n", buffer[0]);
    const uint8_t cap1298_noise_register = 0x0a;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_noise_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("noise: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal1_register = 0xb1;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_cal1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("cal1: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal2_register = 0xb2;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_cal2_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("cal2: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal3_register = 0xb3;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_cal3_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("cal3: %02hhx\n", buffer[0]);
    const uint8_t cap1298_cal4_register = 0xb4;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_cal4_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("cal4: %02hhx\n", buffer[0]);
    const uint8_t cap1298_lsb1_register = 0xb9;
    i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_lsb1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
    printf("lsb1: %02hhx\n", buffer[0]);
}

void turn_motor(uint8_t direction, struct Context context)
{

}

int main() 
{
    struct Context ctx;
    ctx.led = 1;
    uint8_t turn_direction;
    uint8_t turn_flag;
    setup_io();
    update_rgb(ctx);
    setup_rgb();
    setup_power();
    setup_touch();
    #ifdef DEBUG
        print_diagnostics();
    #endif
    uint8_t buffer[1] = {0};
    const uint8_t cap1298_clear_alert[2] = {0x00, 0x00};
    const uint8_t cap1298_button_state_register = 0x03;
    while (true) 
    {   
        if (alert_flag == 1)
        {
            i2c_write_blocking(I2C_PORT, CAP12_ADDR, &cap1298_button_state_register, 1, true);
            i2c_read_blocking(I2C_PORT, CAP12_ADDR, buffer, 1, false);
            i2c_write_blocking(I2C_PORT, CAP12_ADDR, cap1298_clear_alert, 2, false);
            alert_flag = 0;
            switch (buffer[0]){
                case 0x04:
                    ctx.led = !ctx.led;
                    break;
                case 0x02:
                    ctx.enable = !ctx.enable;
                    break;
                case 0x01:
                    turn_flag = 1;
                    turn_direction = 1;
                    break;
                case 0x08:
                    turn_flag = 1;
                    turn_direction = 0;
                    break;
            }
            update_rgb(ctx);
            if (turn_flag)
            {
                turn_motor(turn_direction, ctx);
                turn_flag = 0;
            }
            // printf("button: %02hhx, %u\n", buffer[0], ++button_counter);
        }
    }
    return 0;
}
