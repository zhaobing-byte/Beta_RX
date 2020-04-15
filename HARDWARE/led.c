#include "led.h"


void led_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef PORT_LED;
	PORT_LED.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_4;
	PORT_LED.GPIO_Mode=GPIO_Mode_OUT;
	PORT_LED.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&PORT_LED);
}


void LedToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if(((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET))
	{
		GPIOx->BRR  = GPIO_Pin;
	}
	else
	{
		GPIOx->BSRR = GPIO_Pin;
	}
}
