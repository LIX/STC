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

unsigned char Res_Buf[50]; //接收数据的数组，用来接收串口数据
unsigned char Res_Count=0; //接收数据的字节计数器，表示本次一帧数据包含几个字节
unsigned char Res_Sign=0; //接收到数据标志，接收到1个字节就会置1
unsigned char receive_delay=0; // 延时计数器，用来判断有没有接收完一帧数据
char systick;
unsigned char loop_counter=0;
bit busy; 
bit data_ready;
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
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设定定时器1为16位自动重装方式
	TL1 = T1_TIM;		//设定定时初值
	TH1 = T1_TIM>>8;		//设定定时初值
	ET1 = 0;		//禁止定时器1中断
	ES = 1;
	TR1 = 1;		//启动定时器1
}

void Init_Timer0(void)		//1毫秒@11.0592MHz
{
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = T0_TIM;		//设置定时初值
	TH0 = T0_TIM>>8;	//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0 = 1;		// 使能定时器0溢出中断
}

void Delay1us()		//@11.0592MHz
{
	unsigned char i;
	_nop_();
	_nop_();
	i = 1;
	while (--i);
}

void ISR_UART1() interrupt 4 // 串口中断服务函数
{
	if (TI)
	{
		TI = 0;
		busy = 0;
	}
	if (RI) // 如果接收到一个字节
	{
		RI = 0; // 中断标志位清0
		Res_Buf[Res_Count++]=SBUF; // 把数据保存到接收数组
		Res_Sign=1; // 表示已经接收到数据
		receive_delay=0; // 延时计数器清0
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

void ISR_Timer0() interrupt 1	// 定时器0中断函数
{
	receive_delay++;
	loop_counter++;
	
	if (loop_counter%50 == 0)
	{
		P54=!P54;
		loop_counter=0;
		data_ready=1;
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
// bit6: 关闭REF输出 
// bit5:4 更新速率 0->10Hz 1->40Hz 2->640Hz 3->1280Hz
// bit3:2 PGA 0->1 1->2 2->64 3->128
// bit1:0 通道 0->A 1->保留 2->温度 3->内短
unsigned long SPI_1237(char operation_type, char config)
{
	char i;
	unsigned long data_temp=0;
	unsigned char cmd;
	if (operation_type == read_AD)	// 读AD值
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
		if (operation_type == write_config)	// 写配置
		{
			cmd = SPI_WRITE_CMD;
		}
		else if (operation_type == read_config)  //	读配置
		{
			cmd = SPI_READ_CMD;
		}
		
		// 30~37 bit 发送命令
		cmd <<= 1; //填充成8bit
		for (i=7; i>=0; i--)
		{
			SCLK_H();
			Delay1us();
			DOUT=(cmd>>i) & 0x1;
			SCLK_L();
			Delay1us();
		}
		
		if (operation_type == write_config)	// 写配置
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
		else if (operation_type == read_config)  //	读配置
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
	//	0		1	推挽输出
	//	1		0	高阻输入
	//	1		1	开漏输出
	
	// P3.2->SCLK  P3.3->MISO
	P3M1 &= (~((1<<2)|(1<<3)));	// 准双向口
	P3M0 &= (~((1<<2)|(1<<3)));
	
	P32=1; //利用上拉电阻输出1
	P33=1;
	
	//P_SW1 = 0x80;	// 串口1映射到5.4 5.5
	// 3.0->Rx  3.1->Tx
	P3M1 &= (~((1<<1)|(1<<0)));	// 准双向口
	P3M0 &= (~((1<<1)|(1<<0)));
	
	
	P5M1 &= (~((1<<4)|(1<<5)));	// 准双向口
	P5M0 &= (~((1<<4)|(1<<5)));
}
int main()
{
	
	Init_GPIO();
	
	Init_Uart();
	
	Init_Timer0();
	
	EA = 1;	// 使能总中断
	
	while(1)
	{
//		if (DOUT == 0)
//		{
//			unsigned long temp = SPI_1237(0, 0);
//			UartSendStr((unsigned char*)&temp, 4);
//		}
		
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