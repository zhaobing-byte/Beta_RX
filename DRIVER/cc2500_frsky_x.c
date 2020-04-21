#include "cc2500_frsky_x.h"
#include "rx_cc2500.h"
#include "system.h"
#include "led.h"
#include "function.h"
#include "flash.h"

extern uint16_t rcData[];

static uint8_t protocolState;
static uint8_t calData[255][3];

static uint32_t timeTunedMs;
static int8_t bindOffset;
uint8_t listLength;
static uint8_t bindIdx;

static uint32_t start_time;

typedef rx_spi_received_e handlePacketFn(uint8_t * const packet, uint8_t * const protocolState);
static inline int32_t cmpTimeUs(uint32_t a, uint32_t b) { return (int32_t)(a - b);}
typedef void setRcDataFn(uint16_t *rcData, const uint8_t *payload);

static handlePacketFn *handlePacket;
static setRcDataFn *setRcData;

uint32_t missingPackets;
int32_t timeoutUs;

static uint32_t packetTimerUs;
static bool frameReceived;
//static uint32_t receiveDelayUS;
static int32_t receiveDelayUs;
static uint8_t packetLength;
static uint16_t telemetryDelayUs;

rxCc2500SpiConfig_t rxCc2500SpiConfigMutable = {
		true,                            //autoBind
		{43,198},                        //bindTxId
		0,                               //bindOffset
		{0,0,0,0,0,0,0,0,0,0,            //***************************
         0,0,0,0,0,0,0,0,0,0,            
		 0,0,0,0,0,0,0,0,0,0,             //bindHopData
		 0,0,0,0,0,0,0,0,0,0,
		 0,0,0,0,0,0,0,0,0,0,},          //***************************
		//{1,201,166,131,96,61,26,226,191,156,121,86,51,16,216,181,146,111,76,41,6,206,171,136,101,66,31,231,196,161,126,93,56,21,223,186,151,116,81,48,11,211,178,141,106,71,36,0,0,0},
		0,                               //rxNum
		0,                               //a1Source
		0,                               //chipDetectEnabled
		0,                               //txEnIoTag
		0,                               //lnaEnIoTag
		0,                               //antSelIoTag
};

uint8_t listLength;

const uint16_t crcTable[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
        0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
        0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
        0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
        0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
        0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
        0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
        0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
        0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
        0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
        0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
        0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
        0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
        0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
        0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
        0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
        0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
        0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
        0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
        0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
        0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
        0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
        0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
        0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
        0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
        0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
        0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
        0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
        0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
        0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
        0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};


static void initialise()
{
	cc2500Reset();
	cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
    cc2500WriteReg(CC2500_18_MCSM0,    0x18);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
    cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);
    cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);
    cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
    cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0,  0x7A);
    cc2500WriteReg(CC2500_19_FOCCFG,   0x16);
    cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
    cc2500WriteReg(CC2500_21_FREND1,   0x56);
    cc2500WriteReg(CC2500_22_FREND0,   0x10);
    cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
    cc2500WriteReg(CC2500_26_FSCAL0,   0x11);
    cc2500WriteReg(CC2500_29_FSTEST,   0x59);
    cc2500WriteReg(CC2500_2C_TEST2,    0x88);
    cc2500WriteReg(CC2500_2D_TEST1,    0x31);
    cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
    cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);
    cc2500WriteReg(CC2500_09_ADDR,     0x00);
	
	cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
	cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
	cc2500WriteReg(CC2500_0F_FREQ0,    0x27);
	cc2500WriteReg(CC2500_06_PKTLEN,   0x1E);
	cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
	cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
	cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
	cc2500WriteReg(CC2500_11_MDMCFG3,  0x61);
	cc2500WriteReg(CC2500_12_MDMCFG2,  0x13);
	cc2500WriteReg(CC2500_15_DEVIATN,  0x51);
	
	for(unsigned c = 0; c < 0xFF; c++)
	{
		//calibrate all channels
		cc2500Strobe(CC2500_SIDLE);
		cc2500WriteReg(CC2500_0A_CHANNR,c);
		cc2500Strobe(CC2500_SCAL);
		delay_us(900);
		calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
		calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
		calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
	}
	delay_us(20);
}

void initialiseData(bool inBindState)
{
	cc2500WriteReg(CC2500_0C_FSCTRL0, rxCc2500SpiConfigMutable.bindOffset);
	cc2500WriteReg(CC2500_18_MCSM0, 0x08);
	cc2500WriteReg(CC2500_09_ADDR, inBindState ? 0x03 : rxCc2500SpiConfigMutable.bindTxId[0]);
	cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0D);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
	if (!inBindState)
	{
        cc2500WriteReg(CC2500_03_FIFOTHR,  0x14);
    }
    delay_ms(10);
}

static void initTuneRx(void)
{
    cc2500WriteReg(CC2500_19_FOCCFG, 0x14);
    timeTunedMs = millis();
    bindOffset = -126;
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x08);

    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    cc2500Strobe(CC2500_SRX);
}

static void initGetBind(void)
{
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    delay_us(20); // waiting flush FIFO

    cc2500Strobe(CC2500_SRX);
    listLength = 0;
    bindIdx = 0x05;
}

rx_spi_received_e frSkySpiDataReceived(uint8_t *packet)
{
	rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
	switch (protocolState)
	{
		case STATE_INIT:
			if((millis() - start_time) > 10)
			{
				initialise();
				protocolState = STATE_BIND;
			}
			break;
		case STATE_BIND:                               
			if(rxCc2500SpiConfigMutable.autoBind)     //检查是否已经绑定过
			{
				initTuneRx();
				protocolState = STATE_BIND_TUNING;
			}
			else
			{
				protocolState = STATE_STARTING;
			}
			break;
		case STATE_BIND_TUNING:
			//LedToggle(LED_PORT,Red_LED_PIN);
			Blue_LED_ON;
			if(tuneRx(packet))
			{
				initGetBind();
				initialiseData(true);
				protocolState = STATE_BIND_BINDING1;
			}
			break;
		case STATE_BIND_BINDING1:
			if(getBind1(packet))
			{
				protocolState = STATE_BIND_BINDING2;
			}
			break;
		case STATE_BIND_BINDING2:
			if(getBind2(packet))
			{
				cc2500Strobe(CC2500_SIDLE);
				rxCc2500SpiConfigMutable.autoBind = false;
				protocolState = STATE_BIND_COMPLETE;
			}
			break;
		case STATE_BIND_COMPLETE:
			if(!rxCc2500SpiConfigMutable.autoBind) 
			{
				writeEEPROM();
			}
			else
			{
				uint8_t ctr = 80;
				while(ctr--)
				{
					LedToggle(LED_PORT,Blue_LED_PIN);
					delay_ms(50);
				}
			}
			ret = RX_SPI_RECEIVED_BIND;
			protocolState = STATE_STARTING;
			break;
		default:
			ret = handlePacket(packet,&protocolState);
			break;		
	}
	return ret;
}

static bool tuneRx(uint8_t *packet)
{
	if(bindOffset >= 126)
	{
		bindOffset = -126;
    }
	if((millis() - timeTunedMs) > 50)
	{
		timeTunedMs = millis();
		bindOffset += 5;
		cc2500WriteReg(CC2500_0C_FSCTRL0,(uint8_t)bindOffset);
	}
	if(rxSpiGetExtiState())                                                              //获取CC2500接收输出信号
	{
		uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;      //获取接收到字节的长度
		if(ccLen)
		{
			cc2500ReadFifo(packet, ccLen);
			if(packet[2] == 0x01)
			{
				uint8_t Lqi = packet[ccLen - 1] & 0x7F;
				if(Lqi > 50)
				{
					rxCc2500SpiConfigMutable.bindOffset = bindOffset;
					return true;
				}
			}
		}
	}
	return false;
}

static bool getBind1(uint8_t *packet)
{
	// len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
	if(rxSpiGetExtiState())
	{
		uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
		if(ccLen)
		{
			cc2500ReadFifo(packet, ccLen);
			if (packet[ccLen - 1] & 0x80) 
			{
				 if(packet[2] == 0x01)
				 {
					 if(packet[5] == 0x00)
					 {
					    rxCc2500SpiConfigMutable.bindTxId[0] = packet[3];
                        rxCc2500SpiConfigMutable.bindTxId[1] = packet[4];
						 for (uint8_t n = 0; n < 5; n++)
						 {
                            rxCc2500SpiConfigMutable.bindHopData[packet[5] + n] = packet[6 + n];
                         }
						 rxCc2500SpiConfigMutable.rxNum = packet[12];
                         return true;
					 }
				 }
			}
		}
	}
	return false;
}

static bool getBind2(uint8_t *packet)
{
	if(bindIdx <= 120)
	{
		if(rxSpiGetExtiState())
		{
			uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if(ccLen)
			{
				cc2500ReadFifo(packet, ccLen);

				if(packet[ccLen - 1] & 0x80)
				{
					if(packet[2] == 0x01)
					{
						if ((packet[3] == rxCc2500SpiConfigMutable.bindTxId[0]) && (packet[4] == rxCc2500SpiConfigMutable.bindTxId[1]))
						{
							if(packet[5] == bindIdx)
							{
//#if defined(DJTS)
								if(packet[5] == 0x2D)
								{
									for (uint8_t i = 0; i < 2; i++) 
									{
                                        rxCc2500SpiConfigMutable.bindHopData[packet[5] + i] = packet[6 + i];
                                    }
									listLength = 47;
									return true;
								}
//#endif
								for (uint8_t n = 0; n < 5; n++)
								{
									if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0))
									{
										if (bindIdx >= 0x2D)
										{
											listLength = packet[5] + n;
											return true;
										}
									}
									rxCc2500SpiConfigMutable.bindHopData[packet[5] + n] = packet[6 + n];
								}
								bindIdx = bindIdx + 5;
								return false;
							}
						}
					}
				}
			}
		}
		return false;
	}
	else
	{
		return true;
	}
}

void nextChannel(uint8_t skip)     //frsky_share
{
    static uint8_t channr = 0;

    channr += skip;
    while (channr >= listLength) {
        channr -= listLength;
    }
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3,calData[rxCc2500SpiConfigMutable.bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2,calData[rxCc2500SpiConfigMutable.bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1,calData[rxCc2500SpiConfigMutable.bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, rxCc2500SpiConfigMutable.bindHopData[channr]);
//    if (spiProtocol == RX_SPI_FRSKY_D) {
//        cc2500Strobe(CC2500_SFRX);
//    }
}

rx_spi_received_e frSkyXHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
{
	static bool skipChannels = true;
	static uint8_t channelsToSkip = 1;
	static uint32_t packetErrors = 0;
	static bool telemetryReceived = false;
	rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
	switch (*protocolState)
	{
		case STATE_STARTING:
			listLength = 47;
			initialiseData(false);
			*protocolState = STATE_UPDATE;
			nextChannel(1);
			cc2500Strobe(CC2500_SRX);
			break;
		case STATE_UPDATE:
			 packetTimerUs = micros();
            *protocolState = STATE_DATA;
			frameReceived = false; // again set for receive
			receiveDelayUs = 5300;
			if(!rxSpiCheckBindRequested()) 
			{       
				//检测绑定请求
				packetTimerUs = 0;
				timeoutUs = 50;
				missingPackets = 0;
				*protocolState = STATE_INIT;            //检测到绑定请求，接收机进入初始化
				rxCc2500SpiConfigMutable.autoBind = true;
				break;
			}
		case STATE_DATA:
			if(rxSpiGetExtiState() && (frameReceived == false))
			{
				uint8_t cclen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
				if(cclen >= packetLength)
				{
					cc2500ReadFifo(packet,packetLength);
					if(isValidPacket(packet))
					{
						 //如果包是可用的。。。。
						 missingPackets = 0;
						 timeoutUs = 1;
						 receiveDelayUs = 0;
						 //Blue_LED_ON;
						 LedToggle(LED_PORT,Blue_LED_PIN);
						 if(skipChannels)
						 {
							 channelsToSkip = packet[5] << 2;
							 if(packet[4] >= listLength)
							 {
								if(packet[4] < (64 + listLength))
								{
									channelsToSkip += 1;
								}
								else if(packet[4] < (128 + listLength))
								{
									channelsToSkip += 2;
								}
								else if(packet[4] < (192 + listLength))
								{
									channelsToSkip += 3;
								}
							 }
							 telemetryReceived = true;
							 skipChannels = false;
						 }
						 packetTimerUs = micros();      //如果接收到的包是有效的，记录下当前时间点
						 frameReceived = true;
					}
					if(!frameReceived)
					{
						packetErrors++;
						cc2500Strobe(CC2500_SFRX);
					}
				}
			}
			if(telemetryReceived)
			{
				if(cmpTimeUs(micros(),packetTimerUs) > receiveDelayUs)      //如果收到的数据包是有效的，直接到STATE_TELEMETRY,如果不是有效的等待5300mm才会进入
				{ 	
					// if received or not received in this time sent telemetry data
					*protocolState = STATE_TELEMETRY;
					//*protocolState = STATE_RESUME;
					//buildTelemetryFrame(packet);
				}
			}
			if(cmpTimeUs(micros(),packetTimerUs) > timeoutUs * SYNC_DELAY_MAX)
			{
				//----LED FLASH -----
				//----RSSI SET-------
				nextChannel(1);
				LedToggle(LED_PORT,Blue_LED_PIN);
				cc2500Strobe(CC2500_SRX);
				
				*protocolState = STATE_UPDATE;
			}
			if(frameReceived)
			{
				ret |= RX_SPI_RECEIVED_DATA;
			}
			break;
//#ifdef USE_RX_FRSKY_SPI_TELEMETY
		case STATE_TELEMETRY:
			if(cmpTimeUs(micros(),packetTimerUs) >= receiveDelayUs + telemetryDelayUs)      //53mm   +    400us
			{
				cc2500Strobe(CC2500_SIDLE);
				cc2500SetPower(6);
				cc2500Strobe(CC2500_SFRX);
				delay_us(30);
				cc2500Strobe(CC2500_SIDLE);
				//cc2500WriteFifo(frame,frame[0] + 1);
				*protocolState = STATE_RESUME;
			}
			break;
//#endif
		case STATE_RESUME:
			if(cmpTimeUs(micros(),packetTimerUs) > receiveDelayUs + 3700)
			{
				packetTimerUs = micros();
				receiveDelayUs = 5300;
				frameReceived = false; // again set for receive
				nextChannel(channelsToSkip);
				cc2500Strobe(CC2500_SRX);
				if(missingPackets > MAX_MISSING_PKT)
				{
					timeoutUs = 50;
					skipChannels = true;
					telemetryReceived = false;
					*protocolState = STATE_UPDATE;
					break;
				}
				missingPackets++;
				*protocolState = STATE_DATA;
			}
			break;	
	}//end switch
	return ret;
}

void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet)
{
	uint16_t c[8];
    // ignore failsafe packet
    if (packet[7] != 0) {
        return;
    }
    c[0] = (uint16_t)((packet[10] << 8) & 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11] << 4) & 0xFF0) | (packet[10] >> 4);
    c[2] = (uint16_t)((packet[13] << 8) & 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14] << 4) & 0xFF0) | (packet[13] >> 4);
    c[4] = (uint16_t)((packet[16] << 8) & 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17] << 4) & 0xFF0) | (packet[16] >> 4);
    c[6] = (uint16_t)((packet[19] << 8) & 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20] << 4) & 0xFF0) | (packet[19] >> 4);

    for (unsigned i = 0; i < 8; i++) {
        const bool channelIsShifted = c[i] & 0x800;
        const uint16_t channelValue = c[i] & 0x7FF;
      //  rcData[channelIsShifted ? i + 8 : i] = ((channelValue - 64) * 2 + 860 * 3) / 3;
		rcData[channelIsShifted ? i + 8 : i] = channelValue;
    }
}

void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload)
{
	setRcData(rcData, payload);
}

static uint16_t calculateCrc(const uint8_t *data, uint8_t len) 
{
    uint16_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        crc = (crc << 8) ^ (crcTable[((uint8_t)(crc >> 8) ^ *data++) & 0xFF]);
    }
    return crc;
}


bool isValidPacket(const uint8_t *packet)
{
    uint16_t lcrc = calculateCrc(&packet[3], (packetLength - 7));
    if ((lcrc >> 8) == packet[packetLength - 4] && (lcrc & 0x00FF) == packet[packetLength - 3] &&
        (packet[0] == packetLength - 3) &&
        (packet[1] == rxCc2500SpiConfigMutable.bindTxId[0]) &&
        (packet[2] == rxCc2500SpiConfigMutable.bindTxId[1]) &&
        (rxCc2500SpiConfigMutable.rxNum == 0 || packet[6] == 0 || packet[6] == rxCc2500SpiConfigMutable.rxNum)) {
        return true;
    }
    return false;
}


bool frSkySpiInit(void)
{
	packetLength = 32;
	telemetryDelayUs = 400;
	missingPackets = 0;
    timeoutUs = 50;
	handlePacket = frSkyXHandlePacket;
	setRcData = frSkyXSetRcData;
	start_time = millis();
	protocolState = STATE_INIT;
	return true;
}




