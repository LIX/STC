#include "STC8.h"
#include "intrins.h" 

#define	Fosc	11059200UL
#define T0_frequency	1000
#define	T0_TIM	(65536-(Fosc/1/T0_frequency))

#define	BuadRate	115200
#define	T1_TIM	(65536-(Fosc/4/BuadRate))

#define SPI_READ_CMD	0x56
#define SPI_WRITE_CMD	0x65

#define DOUT	P33
#define DOUT_Mask	3

#define P3INTE (*(unsigned char volatile xdata *)0xfd03)
#define P3INTF (*(unsigned char volatile xdata *)0xfd13)
#define P3IM0 (*(unsigned char volatile xdata *)0xfd23)
#define P3IM1 (*(unsigned char volatile xdata *)0xfd33) 

unsigned char Res_Buf[50]; // receive buffer
unsigned char Res_Count=0; // receive bytes counter
unsigned char Res_Sign=0; // date receive flag
unsigned char receive_delay=0; // receive timeout flag
char systick;
unsigned char loop_counter=0;
bit busy; 
bit data_ready;
bit DOUT_old;
bit CS1237_ready;
unsigned char buffer[5];


void SCLK_H()
{
	P32 = 1;
}

void SCLK_L()
{
	P32 = 0;
}
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
		Res_Buf[Res_Count++]=SBUF; // save data to buffer
		Res_Sign=1; // 
		receive_delay=0; // 
	} 
}

void UartSend(char dat)
{
 while (busy);
 busy = 1;
 SBUF = dat;
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

void ISR_INT1() interrupt 2
{
	EX1=0;
	CS1237_ready=1;
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
//void config_stc8g_DOUT(char type)
//{
//	if (type == input)
//	{
//		P3M0=P3M0&(~(1<<3));
//		P3M1=P3M1|(1<<3);
//	}
//	else if (type == output)
//	{
//		P3M0 = P3M0|(1<<3);
//		P3M1 = P3M1&(~(1<<3));
//	}
//}



// bit7: reserve
// bit6: close REF output
// bit5:4 update frequency 0->10Hz 1->40Hz 2->640Hz 3->1280Hz
// bit3:2 PGA 0->1 1->2 2->64 3->128
// bit1:0 channel 0->A 1->reserve 2->temprature 3->internal short
unsigned long SPI_1237(char operation_type, char config)
{
	char i;
	unsigned long data_temp=0;
	unsigned char cmd;
	if (operation_type == read_AD)	// read AD
	{
		for ( i=0; i<27; i++)
		{
			SCLK_H();
			//Delay1us();
			_nop_();
			data_temp |= DOUT;
			data_temp <<= 1;
			SCLK_L();
			_nop_();
			//Delay1us();
		}
		return  data_temp<<5;
	}
	else 
	{
		// 1~29 bit
		for ( i=0; i<29; i++)
		{
			SCLK_H();
			Delay1us();
			data_temp |= DOUT;
			data_temp <<= 1;
			SCLK_L();
			Delay1us();
		}
		data_temp = data_temp>>5;
		
		// config_stc8g_DOUT(output);
		if (operation_type == write_config)	// write config
		{
			cmd = SPI_WRITE_CMD;
		}
		else if (operation_type == read_config)  //	read config
		{
			cmd = SPI_READ_CMD;
		}
		
		// 30~37 bit config
		cmd <<= 1; //padding to 8bit
		for (i=7; i>=0; i--)
		{
			SCLK_H();
			Delay1us();
			DOUT=(cmd>>i) & 0x1;
			SCLK_L();
			Delay1us();
		}
		
		if (operation_type == write_config)	// write config
		{
			// config_stc8g_DOUT(output);
			for (i=7; i>= 0x0; i--)
			{
				SCLK_H();
				Delay1us();
				DOUT=(config>>i) & 0x1;
				SCLK_L();
				Delay1us();
			}
			// bit46
			SCLK_H();
			Delay1us();
			SCLK_L();
			Delay1us();
			return data_temp<<8;
		}
		else if (operation_type == read_config)  //	read config
		{
			// config_stc8g_DOUT(input);
			for (i=7; i>= 0x0; i--)
			{
				SCLK_H();
				Delay1us();
				data_temp |= DOUT;
				data_temp <<= 1;
				SCLK_L();
				Delay1us();
			}
			// bit46
			SCLK_H();
			Delay1us();
			SCLK_L();
			Delay1us();
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
	P3M1 |= (1<<2)|(1<<3);	// open drain
	P3M0 |= (1<<2)|(1<<3);
	
	
	//P_SW1 |= 0x80;	// maping to 5.4 5.5
	// 3.0->Rx  3.1->Tx
	P3M1 &= (~((1<<1)|(1<<0)));	// 准双向口
	P3M0 &= (~((1<<1)|(1<<0)));
	EX1=1;	// int1 interrupt enable
	IT1=1;	//  interrupt at falling edge
	
	
	P5M1 &= (~((1<<4)|(1<<5)));	// 准双向口
	P5M0 &= (~((1<<4)|(1<<5)));
}
int main()
{
	
	Init_GPIO();
	
	Init_Uart();
	
	Init_Timer0();
	
	EA = 1;	// enable global interrupt
	
	while(1)
	{			
		if (CS1237_ready==1)
		{
			
				unsigned long temp = SPI_1237(0, 0);
			CS1237_ready=0;
				UartSendStr((unsigned char*)&temp, 4);
			EX1=1;
				P54=!P54;
		}
		DOUT_old = DOUT;
		
		if (data_ready==1)
		{
			data_ready=0;
			//SPI_1237(read_config, 0);
			//UartSendStr("Uart Test",9); 
		}
		
//		if(Res_Sign==1) // 如果串口接收到数据
//		{
//			//延时等待接收完一帧数据
//			if (receive_delay>=5)
//			{
//				////////////
//				//这里就可以处理接收数据了
//				////////////
//				Res_Sign=0;	// 接收标志清0
//				Res_Count=0; // 接收数据字节计数器清0				
//			}
//		}
//		
//		if (DOUT == 0)	//	数据转换完成
//		{
//			SPI_1237(read_AD, 0);
//		}
	}
}