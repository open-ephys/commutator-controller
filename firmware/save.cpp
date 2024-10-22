
#include <string.h>
#include <stdio.h>
#include "hardware/flash.h"
#include "save.h"


const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

// static void print_buf(const uint8_t *buf, size_t len) {
//     for (size_t i = 0; i < len; ++i) {
//         printf("%02x", buf[i]);
//         if (i % 16 == 15)
//             printf("\n");
//         else
//             printf(" ");
//     }
// }

void save(const Context &ctx)
{
    uint8_t persist[FLASH_PAGE_SIZE] = {0};
    persist[0] = 'O';
    persist[1] = 'E';
    persist[2] = ctx.led_on ? 0x01 : 0x00;
    persist[3] = ctx.commutator_en ? 0x01 : 0x00;
    flash_range_program(FLASH_TARGET_OFFSET, persist, FLASH_PAGE_SIZE);
    //print_buf(flash_target_contents, FLASH_PAGE_SIZE);
}

void load(Context &ctx)
{
    //print_buf(flash_target_contents, FLASH_PAGE_SIZE);
    if (flash_target_contents[0] == 'O' && flash_target_contents[1] == 'E') {
        ctx.led_on = flash_target_contents[2] ? true : false;
        ctx.commutator_en = flash_target_contents[3] ? true : false;
    }
}

