#include "rx_cc2500.h"
#define NOP 0xFF

uint8_t ret = 0x00;
void cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
	rxSpiReadCommandMulti(CC2500_3F_RXFIFO | CC2500_READ_BURST, NOP, dpbuffer, len);
}

void cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len)
{
	cc2500Strobe(CC2500_SFTX);     //0x3B SFTX
	rxSpiWriteCommandMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST, dpbuffer, len);
	cc2500Strobe(CC2500_STX);
}

void cc2500ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
	rxSpiReadCommandMulti(address, NOP, data, length);
}

void cc2500WriteRegisterMulti(uint8_t address, uint8_t *data,uint8_t length)
{
	rxSpiWriteCommandMulti(address, data, length);
}

uint8_t cc2500ReadReg(uint8_t reg)
{
	uint8_t data;
	SPI_NSS_LOW;
	SPI_WriteByte(reg | 0x80);
	delay_us(1);
	data = SPI_ReadByte();
	SPI_NSS_HIGH;
	return data;
}

void cc2500Strobe(uint8_t address)
{
	SPI_NSS_LOW;
	SPI_WriteByte(address);
	SPI_NSS_HIGH;
}

void cc2500WriteReg(uint8_t address, uint8_t data)
{
	SPI_NSS_LOW;
	SPI_WriteByte(address);
	SPI_WriteByte(data);
	SPI_NSS_HIGH;
}

void cc2500SetPower(uint8_t power)
{
	const uint8_t patable[8] = {
		0xC5,    // -12dbm
		0x97,    // -10dbm
		0x6E,    // -8dbm
		0x7E,    // -6dbm
		0xA9,    // -4dbm
		0xBB,    // -2dbm
		0xFE,    // 0dbm
		0xFF,    // 1.5dbm
	};
	if(power > 7)
		power = 7;
	cc2500WriteReg(CC2500_3E_PATABLE, patable[power]);
}

uint8_t cc2500Reset(void)
{
	cc2500Strobe(CC2500_SRES);
	delay_ms(1);    //1000us
	ret = cc2500ReadReg(CC2500_0E_FREQ1);
	return cc2500ReadReg(CC2500_0E_FREQ1) == 0xC4;       //check if reset
}


