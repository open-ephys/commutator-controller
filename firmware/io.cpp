#include "io.h"

void io_alert_irq_callback(unsigned int gpio, long unsigned int events) { alert_flag = 1; }

void io_init()
{
    // Initialize serial port
    stdio_init_all();
    
    // Initialize GPIO as digital output for LED driver
    gpio_init(IS31FL3193_SDB);
    gpio_set_dir(IS31FL3193_SDB, GPIO_OUT);

    // Initialize GPIO as digital i/o for supercap charger
    gpio_init(LTC4425_EN);
    gpio_set_dir(LTC4425_EN, GPIO_OUT);

    gpio_init(LTC4425_VMID_SEL);
    gpio_set_dir(LTC4425_VMID_SEL, GPIO_OUT);

    gpio_init(LTC4425_nPOW_FAIL);
    gpio_set_dir(LTC4425_nPOW_FAIL, GPIO_IN);

    // Initialize GPIO as analog input for supercap charger
    adc_init();
    adc_select_input(LTC4425_CHARGE_CURR);

    // Initialize GPIO as i2c for cap touch sensor and LED driver
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    // Initialize GPIO as pin interrupt for cap touch sensor
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);

    // Initialize GPIO as digital outputs for stepper motor driver
    gpio_init(TMC2130_CFG3_CS);
    gpio_set_dir(TMC2130_CFG3_CS, GPIO_OUT);
    gpio_put(TMC2130_CFG3_CS, 1);

    gpio_init(TMC2130_DIR);
    gpio_set_dir(TMC2130_DIR, GPIO_OUT);
    gpio_put(TMC2130_DIR, 0);

    gpio_init(TMC2130_STEP);
    gpio_set_dir(TMC2130_STEP, GPIO_OUT);
    gpio_put(TMC2130_STEP, 0);

    gpio_init(TMC2130_CFG6_EN);
    gpio_set_dir(TMC2130_CFG6_EN, GPIO_OUT);
    gpio_set_outover(TMC2130_CFG6_EN, GPIO_OVERRIDE_INVERT);
    
    // Initialize GPIO as SPI for stepper motor driver
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(TMC2130_CFG2_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(TMC2130_CFG0_MISO, GPIO_FUNC_SPI);
    gpio_set_function(TMC2130_CFG1_MOSI, GPIO_FUNC_SPI);
}