/* Clocks, millisecond tick, delays and the watchdog. */
#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32g4xx.h"
#include <stdint.h>

#define F_SYS 170000000u

extern volatile uint32_t ms_ticks;

void system_init(void);
void delay_us(uint32_t us);
void wdg_init(void);

static inline void wdg_kick(void)
{
    IWDG->KR = 0xAAAA;
}

#endif
