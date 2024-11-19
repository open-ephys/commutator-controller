#pragma once

#include "defs.h"

#define IS31_ADDR               0x68

void rgb_init(Context context);
void rgb_set_breathing(bool breathing); 
void rgb_set_blue();
void rgb_set_red(); 
void rgb_set_green();
void rgb_set_auto(Context context);
