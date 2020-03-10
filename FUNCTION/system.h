#ifndef _SYSTEM_H_
#define _SYSTEM_H_
#include "stm32f0xx.h"

//extern volatile uint32_t sysTickUptime;
void cycleCounterInit(void);
uint32_t millis(void);
uint32_t micros(void);
uint32_t microsISR(void);
#endif

