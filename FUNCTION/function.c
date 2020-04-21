#include "function.h"
#include "flash.h"
#include "rx_cc2500.h"
#include <string.h>
extern rxCc2500SpiConfig_t rxCc2500SpiConfigMutable;

void Get_ChipID(union ChipID *chipID)
{
	chipID->ChipUniqueID[0] = *(__IO uint32_t *)(0X1FFFF7AC);  // low byte
	chipID->ChipUniqueID[1] = *(__IO uint32_t *)(0X1FFFF7B8);  // 
	chipID->ChipUniqueID[2] = *(__IO uint32_t *)(0X1FFFF7B4);  // high byte
}

uint16_t GetUniqueID(void)
{
	uint16_t ID = 0 ; 
	union ChipID chipID;
	Get_ChipID(&chipID);
	ID = chipID.IDbyte[0] + chipID.IDbyte[2] + chipID.IDbyte[4] + chipID.IDbyte[6] + chipID.IDbyte[8] + chipID.IDbyte[10];
	ID = (ID << 8) + chipID.IDbyte[1] + chipID.IDbyte[3] + chipID.IDbyte[5] + chipID.IDbyte[7] + chipID.IDbyte[9] + chipID.IDbyte[11];
	return ID;
}

void writeEEPROM(void)
{
	Flash_EreasePage(14,2);
	Flash_WriteDatas(PAGE_14_START_ADRESS,(uint16_t *)&rxCc2500SpiConfigMutable,30);
}

void readEEPROM(void)
{
	uint16_t temp[30];
	FLASH_ReadDatas(PAGE_14_START_ADRESS,temp,30);
	memcpy(&rxCc2500SpiConfigMutable,temp,60);
}


