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

unsigned char Res_Buf[50]; //�������ݵ����飬�������մ�������
unsigned char Res_Count=0; //�������ݵ��ֽڼ���������ʾ����һ֡���ݰ��������ֽ�
unsigned char Res_Sign=0; //���յ����ݱ�־�����յ�1���ֽھͻ���1
unsigned char receive_delay=0; // ��ʱ�������������ж���û�н�����һ֡����
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
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//�趨��ʱ��1Ϊ16λ�Զ���װ��ʽ
	TL1 = T1_TIM;		//�趨��ʱ��ֵ
	TH1 = T1_TIM>>8;		//�趨��ʱ��ֵ
	ET1 = 0;		//��ֹ��ʱ��1�ж�
	ES = 1;
	TR1 = 1;		//������ʱ��1
}

void Init_Timer0(void)		//1����@11.0592MHz
{
	AUXR |= 0x80;		//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = T0_TIM;		//���ö�ʱ��ֵ
	TH0 = T0_TIM>>8;	//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	ET0 = 1;		// ʹ�ܶ�ʱ��0����ж�
}

void Delay1us()		//@11.0592MHz
{
	unsigned char i;
	_nop_();
	_nop_();
	i = 1;
	while (--i);
}

void ISR_UART1() interrupt 4 // �����жϷ�����
{
	if (TI)
	{
		TI = 0;
		busy = 0;
	}
	if (RI) // ������յ�һ���ֽ�
	{
		RI = 0; // �жϱ�־λ��0
		Res_Buf[Res_Count++]=SBUF; // �����ݱ��浽��������
		Res_Sign=1; // ��ʾ�Ѿ����յ�����
		receive_delay=0; // ��ʱ��������0
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

void ISR_Timer0() interrupt 1	// ��ʱ��0�жϺ���
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
// bit6: �ر�REF��� 
// bit5:4 �������� 0->10Hz 1->40Hz 2->640Hz 3->1280Hz
// bit3:2 PGA 0->1 1->2 2->64 3->128
// bit1:0 ͨ�� 0->A 1->���� 2->�¶� 3->�ڶ�
unsigned long SPI_1237(char operation_type, char config)
{
	char i;
	unsigned long data_temp=0;
	unsigned char cmd;
	if (operation_type == read_AD)	// ��ADֵ
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
		if (operation_type == write_config)	// д����
		{
			cmd = SPI_WRITE_CMD;
		}
		else if (operation_type == read_config)  //	������
		{
			cmd = SPI_READ_CMD;
		}
		
		// 30~37 bit ��������
		cmd <<= 1; //����8bit
		for (i=7; i>=0; i--)
		{
			SCLK_H();
			Delay1us();
			DOUT=(cmd>>i) & 0x1;
			SCLK_L();
			Delay1us();
		}
		
		if (operation_type == write_config)	// д����
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
		else if (operation_type == read_config)  //	������
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
	//	0		0	׼˫���
	//	0		1	�������
	//	1		0	��������
	//	1		1	��©���
	
	// P3.2->SCLK  P3.3->MISO
	P3M1 &= (~((1<<2)|(1<<3)));	// ׼˫���
	P3M0 &= (~((1<<2)|(1<<3)));
	
	P32=1; //���������������1
	P33=1;
	
	//P_SW1 = 0x80;	// ����1ӳ�䵽5.4 5.5
	// 3.0->Rx  3.1->Tx
	P3M1 &= (~((1<<1)|(1<<0)));	// ׼˫���
	P3M0 &= (~((1<<1)|(1<<0)));
	
	
	P5M1 &= (~((1<<4)|(1<<5)));	// ׼˫���
	P5M0 &= (~((1<<4)|(1<<5)));
}
int main()
{
	
	Init_GPIO();
	
	Init_Uart();
	
	Init_Timer0();
	
	EA = 1;	// ʹ�����ж�
	
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
		
//		if(Res_Sign==1) // ������ڽ��յ�����
//		{
//			//��ʱ�ȴ�������һ֡����
//			if (receive_delay>=5)
//			{
//				////////////
//				//����Ϳ��Դ������������
//				////////////
//				Res_Sign=0;	// ���ձ�־��0
//				Res_Count=0; // ���������ֽڼ�������0				
//			}
//		}
//		
//		if (DOUT == 0)	//	����ת�����
//		{
//			SPI_1237(read_AD, 0);
//		}
	}
}