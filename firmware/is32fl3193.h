#pragma once

#define IS31_ADDR               0x68
#define RGB_SET_RED             rgb_set_color(255, 0, 0);
#define RGB_SET_GREEN           rgb_set_color(1, 20, 7);

void rgb_set_breathing(bool breathing); 
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_init();
