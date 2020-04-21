#ifndef __SBUS_H_
#define __SBUS_H_

#include "stm32f0xx.h"

#define SERIAL_RX_PIN GPIO_Pin_3
#define SERIAL_RX_PORT GPIOA
#define SERIAL_BAUDRATE 100000
#define SBUS_INVERT 0

#define SERIAL_RX_SOURCE GPIO_PinSource3
#define SERIAL_RX_CHANNEL GPIO_AF_1

#define SBUS_FRAME_SIZE 25
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN)/(SBUS_RANGE_MAX-SBUS_RANGE_MIN))//0.625f
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f)) //874.5f

void sbus_init(void);
void sbus_checkrx(void);
void MakeSbusPackage(uint16_t *Array);
#endif
