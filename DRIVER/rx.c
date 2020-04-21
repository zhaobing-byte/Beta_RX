#include <stdbool.h>
#include <stdint.h>
#include "rx.h"
#include "cc2500_frsky_x.h"

//static bool bindRequested;
//static bool lastBindPinStatus;

bool (*protocolInit)(void);
rx_spi_received_e (*protocolDataReceived)(uint8_t *payload);
void (*protocolSetRcDataFromPayload)(uint16_t *rcData, const uint8_t *payload);

bool rxSpiGetExtiState(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef ExtiState;
	ExtiState.GPIO_Pin=GPIO_Pin_1;
	ExtiState.GPIO_Mode=GPIO_Mode_IN;
	ExtiState.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&ExtiState);
	
	if ((GPIOA->IDR & GPIO_Pin_1) != (uint32_t)Bit_RESET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool rxSpiCheckBindRequested(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef Read_IO_KEY;
	Read_IO_KEY.GPIO_Pin=GPIO_Pin_7;
	Read_IO_KEY.GPIO_Mode=GPIO_Mode_IN;
	Read_IO_KEY.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&Read_IO_KEY);
	if ((GPIOB->IDR & GPIO_Pin_7) != (uint32_t)Bit_RESET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool rxSpiSetProtocol(rx_spi_protocol_e protocol)
{
	switch(protocol)
	{
		case RX_SPI_FRSKY_X: 
			protocolInit = frSkySpiInit;
			protocolDataReceived = frSkySpiDataReceived;
			protocolSetRcDataFromPayload = frSkySpiSetRcData;
			break;
		
		case RX_SPI_FRSKY_D: 
			break;
		
		
		case RX_SPI_SFHSS: 
			break;
		default:
			 return false;
	}
	return true;
}



