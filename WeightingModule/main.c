#include "STC8.h"
#include "intrins.h" 

#define int8 char
#define uint8 unsigned char
#define uint16 unsigned short
#define uint32 unsigned long
#define	Fosc	11059200UL
#define T0_frequency	1000
#define	T0_TIM	(65536-(Fosc/1/T0_frequency))

#define	BuadRate	115200
#define	T1_TIM	(65536-(Fosc/4/BuadRate))

#define SPI_READ_CMD	0x56
#define SPI_WRITE_CMD	0x65

#define DOUT	P33
#define DOUT_Mask	3
#define SCLK	P32
#define ID	(char code *)(0x1ff9)

uint8 Res_Count=0; // receive bytes counter
bit Res_Sign=0; // date receive flag
uint8 receive_delay=0; // receive timeout flag
uint8 systick;
uint8 loop_counter=0;
bit busy; 
bit data_ready;
bit DOUT_old;
bit CS1237_ready;
uint8 buffer[5];
struct frame_s
{
	uint8 receiver_ID;
	uint8 sender_ID;
	uint8 length;
	uint8 data_begin[20];
	
};
struct frame_s frame;

extern unsigned int GetCRC16(unsigned char *, unsigned char);

void Init_Uart(void)		//9600bps@11.0592MHz
{
	SCON = 0x50;		//8 bit, adjustable buadrate
	AUXR |= 0x40;		//timer1 frequency = Fosc
	AUXR &= 0xFE;		//
	TMOD &= 0x0F;		// 16 bit auto reload
	TL1 = T1_TIM;		//tiemer low 8 bit
	TH1 = T1_TIM>>8;		//timer high 8 bit
	ET1 = 0;		//disable timer1 interrupt
	ES = 1;
	TR1 = 1;		//enable timer 
	
	//PSH=1;	// interrupt priority 3
	IPH |= (1<<4);
	//PS=1;
	IP |= (1<<4);
}

void Init_Timer0(void)		//1ms @11.0592MHz
{
	AUXR |= 0x80;		
	TMOD &= 0xF0;		
	TL0 = T0_TIM;		
	TH0 = T0_TIM>>8;	
	TF0 = 0;		//clear TF0 
	TR0 = 1;		//timer0 enable
	ET0 = 1;		// enable timer0 interrupt
}

void Delay1us()		//@11.0592MHz
{
	unsigned char i;
	_nop_();
	_nop_();
	i = 1;
	while (--i);
}

void ISR_UART1() interrupt 4 // uart ISR
{
	if (TI)
	{
		TI = 0;
		busy = 0;
	}
	if (RI) 
	{
		RI = 0; 
		frame.data_begin[Res_Count++]=SBUF; // save data to buffer
		Res_Sign=1; // 
		receive_delay=0; // 
	} 
}

void UartSend(char dat)
{
 while (busy);
 SBUF = dat;
 busy = 1;
} 

void UartSendStr(char *p, char count)
{
 while (count-->0)
 {
 UartSend(*p++);
 }
}


void ISR_Timer0() interrupt 1	// timer0 ISR
{
	receive_delay++;
	loop_counter++;
	if (loop_counter%50 == 0)
	{
		loop_counter=0;
		data_ready=1;
	}
}

bit spi_begin = 1;
void ISR_INT1() interrupt 2
{
	//P54=!P54;
	if (spi_begin == 1)
	{
		CS1237_ready=1;
	}
}

enum
{
	read_AD=0,
	write_config,
	read_config
};

enum
{
	output = 0,
	input 
};


// P3.2 SCLK		P3.3 DOUT
void config_stc8g_DOUT(char type)
{
	if (type == input)
	{
		P3M1 |= (1<<3);	// open drain
		P3M0 |= (1<<3);
		P33=1;
	}
	else if (type == output)
	{
		P33=0;
		P3M1 &= ~(1<<3);	// pull push
		P3M0 |= (1<<3);
	}
}

void IapIdle() {
  IAP_CONTR = 0;    //关闭 IAP 功能
  IAP_CMD = 0;      //清除命令寄存器
  IAP_TRIG = 0;     //清除触发寄存器
  IAP_ADDRH = 0x80; //将地址设置到非 IAP 区域
  IAP_ADDRL = 0;
}
char IapRead(int addr) {
  char dat;

  IAP_CONTR = 0x80;      //使能 IAP
  IAP_TPS = 12;          //设置擦除等待参数 12MHz
  IAP_CMD = 1;           //设置 IAP 读命令
  IAP_ADDRL = addr;      //设置 IAP 低地址
  IAP_ADDRH = addr >> 8; //设置 IAP 高地址
  IAP_TRIG = 0x5a;       //写触发命令(0x5a)
  IAP_TRIG = 0xa5;       //写触发命令(0xa5)
  _nop_();
  dat = IAP_DATA; //读 IAP 数据
  IapIdle();
  return dat;
}
void IapProgram(int addr, char dat) {
  IAP_CONTR = 0x80;      //使能 IAP
  IAP_TPS = 12;          //设置擦除等待参数 12MHz
  IAP_CMD = 2;           //设置 IAP 写命令
  IAP_ADDRL = addr;      //设置 IAP 低地址
  IAP_ADDRH = addr >> 8; //设置 IAP 高地址
  IAP_DATA = dat;        //写 IAP 数据
  IAP_TRIG = 0x5a;       //写触发命令(0x5a)
  IAP_TRIG = 0xa5;       //写触发命令(0xa5)
  _nop_();
  IapIdle(); //关闭 IAP 功能
}
void IapErase(int addr) {
  IAP_CONTR = 0x80;      //使能 IAP
  IAP_TPS = 12;          //设置擦除等待参数 12MHz
  IAP_CMD = 3;           //设置 IAP 擦除命令
  IAP_ADDRL = addr;      //设置 IAP 低地址
  IAP_ADDRH = addr >> 8; //设置 IAP 高地址
  IAP_TRIG = 0x5a;       //写触发命令(0x5a)
  IAP_TRIG = 0xa5;       //写触发命令(0xa5)
  _nop_();               //
  IapIdle();             //关闭 IAP 功能
}

// bit7: reserve
// bit6: close REF output
// bit5:4 update frequency 0->10Hz 1->40Hz 2->640Hz 3->1280Hz
// bit3:2 PGA 0->1 1->2 2->64 3->128
// bit1:0 channel 0->A 1->reserve 2->temprature 3->internal short
unsigned long SPI_1237(char operation_type, char config)
{
	char i;
	uint32 data_temp=0;
	uint8 cmd;
	if (operation_type == read_AD)	// read AD
	{
		for ( i=0; i<27; i++)
		{
			SCLK=1;
			//Delay1us();
			_nop_();
			data_temp |= DOUT;
			data_temp <<= 1;
			SCLK=0;
			_nop_();
		}
		return  data_temp<<5;
	}
	else 
	{
		// 1~29 bit
		for ( i=0; i<29; i++)
		{
			SCLK=1;
			_nop_();
			data_temp |= DOUT;
			data_temp <<= 1;
			SCLK=0;
			_nop_();
		}
		data_temp = data_temp>>5;
		
		
		if (operation_type == write_config)	// write config
		{
			cmd = SPI_WRITE_CMD;
		}
		else if (operation_type == read_config)  //	read config
		{
			cmd = SPI_READ_CMD;
		}
		
		// 30~36 bit config
		
		config_stc8g_DOUT(output);
		for (i=6; i>=0; i--)
		{
			SCLK=1;
			_nop_();
			DOUT=(cmd>>i) & 0x1;
			SCLK=0;
			_nop_();
		}
		config_stc8g_DOUT(input);
		
			SCLK=1;
			Delay1us();
			SCLK=0;
			_nop_();
		
		if (operation_type == write_config)	// write config
		{
			config_stc8g_DOUT(output);
			for (i=7; i>= 0x0; i--)
			{
				SCLK=1;
				_nop_();
				DOUT=(config>>i) & 0x1;
				SCLK=0;
				_nop_();
			}
			// bit46
			SCLK=1;
			Delay1us();
			SCLK=0;
			_nop_();
			config_stc8g_DOUT(input);
			return data_temp<<8;
		}
		else if (operation_type == read_config)  //	read config
		{
			// config_stc8g_DOUT(input);
			for (i=7; i>= 0x0; i--)
			{
				SCLK=1;
				_nop_();
				data_temp |= DOUT;
				data_temp <<= 1;
				SCLK=0;
				_nop_();
			}
			// bit46
			SCLK=1;
			Delay1us();
			SCLK=0;
			_nop_();
			return data_temp;
		}
	}
}


void Init_GPIO()
{
	// PnM1.x PnM0.x
	//	0		0	准双向口
	//	0		1	push pull
	//	1		0	high z 
	//	1		1	open drain
	
	// P3.2->SCLK  P3.3->MISO
	
	SCLK=0;	// pull SCLK to previent CS1237 enter low power mode
	P3M1 &= ~(1<<2);	// pull push
	P3M0 |= (1<<2);
	
	
	P3M1 |= (1<<3);	// open drain
	P3M0 |= (1<<3);
	
	//P_SW1 |= 0x80;	// maping to 5.4 5.5
	// 3.0->Rx  3.1->Tx
//	P3M1 &= (~((1<<1)|(1<<0)));	// 准双向口
//	P3M0 &= (~((1<<1)|(1<<0)));
	
	EX1=1;	// int1 interrupt enable
	IT1=1;	//  interrupt at falling edge
	
	
	P5M1 &= (~((1<<4)|(1<<5)));	// 准双向口
	P5M0 &= (~((1<<4)|(1<<5)));
}
	char i;
	unsigned long read_CS1237=0;
	char CS1237_mode=1;
int main()
{
	uint8 eeprom_temp[10];
	Init_GPIO();
	
	Init_Uart();
	
	Init_Timer0();
	
	EA = 1;	// enable global interrupt
	while(1)
	{			
		
		if (CS1237_ready==1)
		{
			unsigned long temp=0;
			CS1237_ready=0;
			spi_begin = 0;
			if (CS1237_mode == 1) {
				read_CS1237 = SPI_1237(read_config, 0x0);
			} else if (CS1237_mode == 2) {
				read_CS1237 =
						SPI_1237(write_config, frame.data_begin[1]);
			}
			spi_begin = 1;
                }

		if(Res_Sign==1) // 如果串口接收到数据
		{
			//延时等待接收完一帧数据
			if (receive_delay>=5)
			{
				////////////
				//这里就可以处理接收数据了
				////////////

				
				uint8 *p_frame_buffer=(uint8 *)&frame;
//			unsigned int crc;
//			unsigned char crch, crcl;
//				    //?????,??????????
//				crc = GetCRC16(p_frame_buffer, Res_Count-2); //?? CRC ???
//				crch = crc >> 8;
//				crcl = crc & 0xFF;
//				if ((p_frame_buffer[Res_Count-2]!=crch) || (p_frame_buffer[Res_Count-1]!=crcl)){
//						return; //? CRC ?????????
//				}
				
				switch(frame.data_begin[0])
				{
					case 1:	
						CS1237_mode = 1;	// read channelA
					break;
					
					case 2:
						CS1237_mode = 2;	// config CS1237
					break;
					
					case 3:
						//read muc ID
						UartSendStr(ID, 7);
					break;

					case 4:	// write eeprom
//						for (i=0; i<10; i++)
//						{
//							eeprom_temp[i]=IapRead(0x0400+i);
//						}
//						eeprom_temp[2]=0x9;
//						IapErase(0x0400);
						for (i=0; i<10; i++)
						{
							IapProgram(0x0400+i, i);
						}
						break;
						
					case 5:	// read eeprom
						for (i=0; i<10; i++)
						{
							eeprom_temp[i]=IapRead(0x0400+i);
						}
						UartSendStr((uint8 *)&eeprom_temp, 10);
						break;
						
					case 6:	// erase eeprom
						for (i=0; i<8; i++)
						{
							IapErase(0x200*i);
						}
						break;
						
					case 7:
					{
						uint16 max_time = frame.data_begin[2];
						uint16 wait_time = GetCRC16(ID, 7)/max_time;
						
						//wait_time = 
					}
						// UartSendStr((uint8 *)&read_CS1237,
						// 4);
					}
				
				Res_Sign=0;	// 接收标志清0
				Res_Count=0; // 接收数据字节计数器清0			
                        }
		}
	}
}