#pragma once

#include "device.h"

// Delay by the given number of milliseconds. This is quite precise since
// it uses the core timer.
void delay_ms(uint32_t amount);

