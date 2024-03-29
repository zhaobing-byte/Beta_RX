#include "sbus.h"
#include "string.h"
#define RX_BUFF_SIZE 64							//SPEK_FRAME_SIZE 16  

uint8_t SbusDataTx[25];

uint8_t rx_buffer[RX_BUFF_SIZE];    //spekFrame[SPEK_FRAME_SIZE]
uint8_t rx_start = 0;
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];		
uint16_t Channel_DataBuff[16];
int stat_overflow;

int framestarted = -1;
uint8_t framestart = 0;
// enable statistics
const int sbus_stats = 0;

unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[25];

// statistics
int stat_framestartcount;
int stat_timing_fail;
int stat_garbage;
//int stat_timing[25];
int stat_frames_accepted = 0;
int stat_frames_second;
int stat_overflow;
uint8_t data[25];

int failsafe = 0;
int rxmode = 0;
int rx_ready = 0;
int channels[16];
extern uint16_t Channel_DataBuff[];
void sbus_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = SERIAL_RX_PIN;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //复用推挽输出
   // GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
	
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure); 
  //  GPIO_PinAFConfig(SERIAL_RX_PORT, SERIAL_RX_SOURCE , SERIAL_RX_CHANNEL);
	GPIO_PinAFConfig(SERIAL_RX_PORT, GPIO_PinSource2 , SERIAL_RX_CHANNEL);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
	
	if(SBUS_INVERT) 
	{
		USART_InvPinCmd(USART1, USART_InvPin_Rx|USART_InvPin_Tx , ENABLE );
	}

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	framestarted = 0;
}

void USART1_IRQHandler(void)
{
    rx_buffer[rx_end] = USART_ReceiveData(USART1);
    // calculate timing since last rx
    unsigned long  maxticks = SysTick->LOAD;	
    unsigned long ticks = SysTick->VAL;	
    unsigned long elapsedticks;	    //逝去节拍数
    static unsigned long lastticks; //上一次的节拍
    if (ticks < lastticks)          //如果当前的节拍数小于小于上次记录的节拍数
	{
        elapsedticks = lastticks - ticks;	   //那么逝去的节拍数就等于 上次记录的节拍数-去现在记录的节拍数
	}
    else
    {// overflow ( underflow really)
		elapsedticks = lastticks + ( maxticks - ticks);   //当前获取的节拍数大于上次的节拍数，说明计数器已经溢出从新开始	
    }

    if ( elapsedticks < 65536 ) 
	{
		rx_time[rx_end] = elapsedticks; //
	}
    else 
	{
		rx_time[rx_end] = 65535;  //0xffff
	}
	
    lastticks = ticks;
       
    if ( USART_GetFlagStatus(USART1 , USART_FLAG_ORE ) )    //数据溢出中断
    {
		// overflow means something was lost 
		rx_time[rx_end]= 0xFFFe;
		USART_ClearFlag( USART1 , USART_FLAG_ORE );     //清除溢出中断标志位
		if ( sbus_stats ) 
		{
		  stat_overflow++;
		}
    }        
    rx_end++;
    rx_end%=(RX_BUFF_SIZE);            //构建循环缓存
}

void sbus_checkrx(void)
{
	if ( framestarted == 0)
	{
		while (  rx_end != rx_start )            //判断缓冲区是否为空，当队列头等于队列尾的时候  说明缓冲区为空
		{ 
			if ( rx_buffer[rx_start] == 0x0f )    //如果不为空，那么我们就要先找到 SBUS 协议的头  0x0f
			{
				// start detected
				framestart = rx_start;            //找到头   开始接收
				framestarted = 1;                 //启动数据接收
				stat_framestartcount++; 
				break;                
			}         
			rx_start++;
			rx_start%=(RX_BUFF_SIZE);           
			stat_garbage++;
		}          
	}
	else if ( framestarted == 1)
	{
		// frame already has begun
		int size = 0;
		if (rx_end > framestart )
		{			
			size = rx_end - framestart;
		}
		else 
		{
			size = RX_BUFF_SIZE - framestart + rx_end;
		}
		
		if ( size >= 24 )
		{    
			int timing_fail = 0; 	
			for ( int i = framestart ; i <framestart + 25; i++  )  
			{
				data[ i - framestart] = rx_buffer[i%(RX_BUFF_SIZE)];
				int symboltime = rx_time[i%(RX_BUFF_SIZE)];         //rx_time 里存储了  每次接收数据的时间间隔
				//stat_timing[ i - framestart] = symboltime;
				if ( symboltime > 0x1690 &&  i - framestart > 0 ) 
				{
					timing_fail = 1;
				}
			}    

			if (!timing_fail) 
			{
				frame_received = 1;  
				last_byte = data[24];
				rx_start = rx_end;
				framestarted = 0;
				bind_safety++;
			} // end frame complete  
			
			rx_start = rx_end;
			framestarted = 0;
		
		}// end frame pending
		else
		{
			if ( framestarted < 0)
			{
				// initialize sbus
				sbus_init();
			   // set in routine above "framestarted = 0;"    
			}
		}
		  
		if ( frame_received )
		{ 		
			Channel_DataBuff[0]  = ((data[1]  |    data[2]<<8)                  & 0x07FF);
			Channel_DataBuff[1]  = ((data[2]  >>3 |data[3]<<5)                  & 0x07FF);
			Channel_DataBuff[2]  = ((data[3]  >>6 |data[4]<<2   |data[5]<<10)   & 0x07FF);
			Channel_DataBuff[3]  = ((data[5]  >>1 |data[6]<<7)                  & 0x07FF);
			Channel_DataBuff[4]  = ((data[6]  >>4 |data[7]<<4)                  & 0x07FF);
			Channel_DataBuff[5]  = ((data[7]  >>7 |data[8]<<1   |data[9]<<9)    & 0x07FF);
			Channel_DataBuff[6]  = ((data[9]  >>2 |data[10]<<6)                 & 0x07FF);
			Channel_DataBuff[7]  = ((data[10] >>5 |data[11]<<3)                 & 0x07FF);
			Channel_DataBuff[8]  = ((data[12] |    data[13]<<8)                 & 0x07FF);
			Channel_DataBuff[9]  = ((data[13] >>3 |data[14]<<5)                 & 0x07FF);
			Channel_DataBuff[10] = ((data[14] >>6 |data[15]<<2  |data[16]<<10)  & 0x07FF);
			Channel_DataBuff[11] = ((data[16] >>1 |data[17]<<7)                 & 0x07FF);
			Channel_DataBuff[12] = ((data[17] >>4 |data[18]<<4)                 & 0x07FF);
			Channel_DataBuff[13] = ((data[18] >>7 |data[19]<<1  |data[20]<<9)   & 0x07FF);
			Channel_DataBuff[14] = ((data[20] >>2 |data[21]<<6)                 & 0x07FF);
			Channel_DataBuff[15] = ((data[21] >>5 |data[22]<<3)                 & 0x07FF);

			frame_received = 0;    
		} // end frame received
	}
}


void MakeSbusPackage(uint16_t *Array)
{
	SbusDataTx[0]  = 0x0f;
	SbusDataTx[1]  =  Array[0]  & 0xff;
	SbusDataTx[2]  = (Array[1]  << 3 | Array[0]  >> 8  ) & 0xff;
	SbusDataTx[3]  = (Array[2]  << 6 | Array[1]  >> 5  ) & 0xff;
	SbusDataTx[4]  = (Array[2]  >> 2 ) & 0xff;
	SbusDataTx[5]  = (Array[3]  << 1 | Array[2]  >> 10 ) & 0xff;
	SbusDataTx[6]  = (Array[4]  << 4 | Array[3]  >> 7  ) & 0xff;
	SbusDataTx[7]  = (Array[5]  << 7 | Array[4]  >> 4  ) & 0xff;
	SbusDataTx[8]  = (Array[5]  >> 1 ) & 0xff;
	SbusDataTx[9]  = (Array[6]  << 2 | Array[5]  >> 9  ) & 0xff;
	SbusDataTx[10] = (Array[7]  << 5 | Array[6]  >> 6  ) & 0xff;
	SbusDataTx[11] = (Array[7]  >> 3 ) & 0xff;
	SbusDataTx[12] = (Array[8]  >> 0 ) & 0xff;
	SbusDataTx[13] = (Array[9] << 3 | Array[8]  >> 8  ) & 0xff;
	SbusDataTx[14] = (Array[10] << 8 | Array[9] >> 5  ) & 0xff;
	SbusDataTx[15] = (Array[10] >> 2 ) & 0xff;
	SbusDataTx[16] = (Array[11] << 1 | Array[10] >> 10 ) & 0xff;
	SbusDataTx[17] = (Array[12] << 4 | Array[11] >> 7  ) & 0xff;
	SbusDataTx[18] = (Array[13] << 7 | Array[12] >> 4 ) & 0xff;
	SbusDataTx[19] = (Array[13] >> 1 ) & 0xff;
	SbusDataTx[20] = (Array[14] << 2 | Array[13] >> 9 ) & 0xff;
	SbusDataTx[21] = (Array[15] << 5 | Array[14] >> 6 ) & 0xff;
	SbusDataTx[22] = (Array[15] >> 3 ) & 0xff;
	SbusDataTx[23] = 0x00;
	SbusDataTx[24] = 0x00;   
		
	for (uint8_t i=0;i<25;i++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
		USART_SendData(USART1, (uint16_t)SbusDataTx[i]);     
	}
}
