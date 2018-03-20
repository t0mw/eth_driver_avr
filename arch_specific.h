#ifndef ARCH_SPECIFIC_H
#define ARCH_SPECIFIC_H

#include "arch_specific.h"
#include "bit_types.h"

// #define DELAY_WAIT_US(x)     _delay_us(x);
#define DELAY_WAIT_US(x)        p_assert(0);

void p_assert(const uint8_t var);

#endif
