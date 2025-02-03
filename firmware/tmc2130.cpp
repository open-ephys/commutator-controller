#include <cstring>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "tmc2130.h"

#define WRITE_FLAG              (1<<7)
#define READ_FLAG               (0<<7)

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(TMC2130_CFG3_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(TMC2130_CFG3_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void tmc2130_write(uint8_t reg, uint32_t data)
{
    uint8_t buf[5] = {(uint8_t)(WRITE_FLAG | reg),
                      (uint8_t)((data >> 24UL) & 0xFF),
                      (uint8_t)((data >> 16UL) & 0xFF),
                      (uint8_t)((data >> 8UL) & 0xFF),
                      (uint8_t)((data >> 0UL) & 0xFF)};


    cs_select();
    spi_write_blocking(TMC2130_SPI_PORT, buf, 5);
    cs_deselect();
}

static uint32_t tmc2130_read(uint8_t reg)
{
    uint8_t dst[5];

    cs_select();
    spi_read_blocking(TMC2130_SPI_PORT, (uint8_t)(READ_FLAG | reg), dst, 5);
    cs_deselect();

    return  (uint32_t)dst[1] << 24 | (uint32_t)dst[2] << 16 | (uint32_t)dst[3] << 8 | (uint32_t)dst[4] << 0;

}

void tmc2130_init()
{
    gpio_put(TMC2130_CFG3_CS, 1);
    gpio_put(TMC2130_DIR, 0);
    gpio_put(TMC2130_STEP, 0);


    // Copied from page 84
    //tmc2130_write(REG_CHOPCONF, 0x05008008UL);
    tmc2130_write(REG_CHOPCONF, 0x050100C3); // TOFF=3, HSTRT=4, HEND=1, CHM=0 (SpreadCycle), TBL=2, VSENSE=0, MRES=8, INTPOL=0
    tmc2130_write(REG_IHOLD_IRUN, 0x00061313); // IHOLD = 0x13 (275 mA), IRUN = 0x13 (275 mA RMS)
    tmc2130_write(REG_TPOWERDOWN, 0x0000000A);
    tmc2130_write(REG_GCONF, 0x00000004);
    tmc2130_write(REG_PWMCONF, 0x000401C8);

    // Start in disabled state
    tmc2130_enable(true);
}

int32_t tmc2130_status()
{
    return tmc2130_read(REG_DRVSTATUS);
}