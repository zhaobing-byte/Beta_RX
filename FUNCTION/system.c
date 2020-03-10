#include "system.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "led.h"
static volatile uint32_t sysTickUptime;
static volatile uint32_t sysTickValStamp;
static volatile uint32_t sysTickPending;
static uint32_t usTicks;

void cycleCounterInit(void)
{
	usTicks = 48000000 / 1000000;               //The value of 1ms per 1us
	SysTick_Config(48000);
}

uint32_t microsISR(void)
{
	register uint32_t ms, pending, cycle_cnt;
	cycle_cnt = SysTick->VAL;
	 if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
	 {
		 sysTickPending = 1;
		 cycle_cnt = SysTick->VAL;
	 }
	 ms = sysTickUptime;
	 pending = sysTickPending;
	 return ((ms + pending)*1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do{
		ms = sysTickUptime;
		cycle_cnt = SysTick->VAL;
	}while(ms != sysTickUptime || cycle_cnt > sysTickValStamp);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t millis(void)
{
    return sysTickUptime;
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	sysTickUptime++;                    //1ms Systick Uptime
	sysTickValStamp = SysTick->VAL;     //1ms SysTick Stamp Value
	sysTickPending = 0;
	static uint8_t x=0;
	if(x == 0)
	{
		LED_ON;
		x = 1;
	}
	else
	{
		LED_OFF;
		x = 0;
	}
}

