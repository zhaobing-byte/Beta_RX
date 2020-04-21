
/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
 /*
		Receiving a request    PB1
 */

  

#include "main.h"
#include "stdio.h"
#include "spi.h"
#include "cc2500_frsky_x.h"
#include "rx.h"
#include "led.h"
#include "system.h"
#include "function.h"
#include "sbus.h"
uint8_t packet[32];
uint16_t rcData[16];
uint32_t cyc_cnt_test;
int main(void)
{
	delay_init(48);
	cycleCounterInit();          //Init cycle Counter
	spi_init();
	rxSpiSetProtocol(RX_SPI_FRSKY_X);
	frSkySpiInit();
	led_Init();
	sbus_init();
	readEEPROM();
	while (1)
	{	
		if((frSkySpiDataReceived(packet) & RX_SPI_RECEIVED_DATA) == 0x02)
		{
			frSkyXSetRcData(rcData,packet);
			MakeSbusPackage(rcData);
		}
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
