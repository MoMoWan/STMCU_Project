#include <stm32f10x_lib.h>
#include "key.h"
#include "delay.h"
//Mini STM32������
//�������� ��������			 
//����ԭ��@ALIENTEK
//2010/5/27   

//������ʼ������
//�ر�ע�⣺�ڸú���֮��JTAG���޷�ʹ�ã�SWDҲ�޷�ʹ�ã�
//�����JTAG���棬�������θú�����
//PA0.13.15 ���ó�����
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	RCC->APB2ENR|=1<<4;     //ʹ��PORTCʱ��
	RCC->APB2ENR|=1<<5;     //ʹ��PORTDʱ��

	GPIOA->CRL&=0XFFFFFFF0;//PA0���ó�����	  
	GPIOA->CRL|=0X00000008;   
	GPIOA->CRH&=0XFFFFFFF0;//PA8���ó�����	  
	GPIOA->CRH|=0X00000008; 				   
	GPIOA->ODR|=0x101;	   //PA0,A8����

	GPIOC->CRH&=0XFF0FFFFF;//PC13���ó�����	  
	GPIOC->CRH|=0X00800000; 				   
	GPIOC->ODR|=1<<13;	   //PC13����

	GPIOD->CRL&=0XFFFF0FFF;//PD3���ó�����	  
	GPIOD->CRL|=0X00008000;    				   
	GPIOD->ODR|=1<<3;	   //PD3����
} 
//����������
//���ذ���ֵ
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2!!
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//�������ɿ���־	

	if(key_up&&(KEY1==0||KEY2==0||KEY3==0||KEY4==0))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY1==0)
		{
			return 1;
		}
		else if(KEY2==0)
		{
			return 2;
		}
		else if(KEY3==0)
		{
			return 3;
		}
		else if(KEY4==0)
		{
			return 4;
		}
	}else if(KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1; 	    
	return 0;// �ް�������
}




















