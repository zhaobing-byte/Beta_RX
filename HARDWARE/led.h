#ifndef __LED_H_
#define __LED_H_

#include "stm32f0xx.h"

#define LED_PORT       GPIOB
#define Blue_LED_PIN   GPIO_Pin_3
#define Red_LED_PIN    GPIO_Pin_4 
 
#define Blue_LED_OFF   (LED_PORT->BRR  = Blue_LED_PIN)
#define Blue_LED_ON    (LED_PORT->BSRR = Blue_LED_PIN)

#define Red_LED_OFF	   (LED_PORT->BRR  = Red_LED_PIN)
#define Red_LED_ON     (LED_PORT->BSRR = Red_LED_PIN)

void led_Init(void);
void LedToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
#endif

