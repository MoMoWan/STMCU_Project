/*************************************************
STM32 ϵͳsystick��ȷ��ʱ֧���ļ�

MCU��STM32F103CBT6���⺯���汾��3.5.0 KEIL

������2013-01-16

*************************************************/


#include "stm32f10x.h"
#include "delay.h"
static u8   fac_us=0;//us��ʱ������ 
static u16 fac_ms=0;//ms��ʱ������ 

void Delay_s(u16 i)
{ u16 t;
  
	while(i)
	{
	for (t=65535;t>0;t--);
  i--;
  }


}





//----------------------������ʱ----------------------------------------//
void delay_ms(u16 nms) 
{        
u32 temp; 
SysTick->CTRL&=0xfffffffb; //ѡ���ڲ�ʱ�� HCLK/8 
fac_us=8;       
fac_ms=(u16)fac_us*1000;
     
SysTick->LOAD=(u32)nms*fac_ms;//ʱ����� 
//SysTick_SetReload((u32)nms*fac_ms); 
SysTick->VAL =0x00;            //��ռ����� 
//SysTick_CounterCmd(SysTick_Counter_Clear); 
SysTick->CTRL=0x01 ;           //��ʼ���� 
//SysTick_CounterCmd(SysTick_Counter_Enable); 
do 
{ 
   temp=SysTick->CTRL; 
} 
while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��    
SysTick->CTRL=0x00;        //�رռ����� 
SysTick->VAL =0x00;        //��ռ�����   
      
} 
//----------------------΢����ʱ----------------------------------------//
void delay_us(u32 us) 
{         
u32 temp; 

SysTick->CTRL&=0xfffffffb;	//ѡ���ڲ�ʱ�� HCLK/8 
fac_us=8;       
fac_ms=(u16)fac_us*1000;

     
SysTick->LOAD=(u32)us*fac_us;//ʱ����� 
//SysTick_SetReload((u32)nms*fac_ms); 
SysTick->VAL =0x00;            //��ռ����� 
//SysTick_CounterCmd(SysTick_Counter_Clear); 
SysTick->CTRL=0x01 ;           //��ʼ���� 
//SysTick_CounterCmd(SysTick_Counter_Enable); 
do 
{ 
   temp=SysTick->CTRL; 
} 
while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��    
SysTick->CTRL=0x00;        //�رռ����� 
SysTick->VAL =0x00;        //��ռ�����   
      
} 
