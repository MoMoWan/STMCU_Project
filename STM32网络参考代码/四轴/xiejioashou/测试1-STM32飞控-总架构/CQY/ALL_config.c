#include "stm32f10x.h"

#include "ALL_config.h"
NVIC_InitTypeDef NVIC_InitStructure;

void RCC_Configuration(void)
{
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

  FLASH_SetLatency(FLASH_Latency_2);//����flash��ʱ
	/*  ʹ���ڲ�RC���� */
   RCC_HSICmd(ENABLE);//ʹ���ڲ����پ��� ;    
	 RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);//PLL����=16
   RCC_PLLCmd(ENABLE);
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//ѡ��PLLʱ����Ϊϵͳʱ��SYSCLOCK=64MHZ 
   RCC_HCLKConfig(RCC_SYSCLK_Div1);//ѡ��HCLKʱ��ԴΪϵͳʱ��SYYSCLOCK
   RCC_PCLK1Config(RCC_HCLK_Div4);//APB1ʱ��64/4
   RCC_PCLK2Config(RCC_HCLK_Div4);//APB2ʱ��Ϊ64/4

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}



void Tim2_init()//PWM����
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	TIM2->ARR=1600;//�趨�������Զ���װֵ
	TIM2->PSC=0;//Ԥ��Ƶ������Ƶ
	TIM2->CCMR1|=6<<4; //CH1 PWM2ģʽ
	TIM2->CCMR1|=1<<3; //CH1Ԥװ��ʹ��
	TIM2->CCMR1|=6<<12; //CH2 PWM2ģʽ
	TIM2->CCMR1|=1<<11; //CH2Ԥװ��ʹ��
	TIM2->CCMR2|=6<<4; //CH3 PWM2ģʽ
	TIM2->CCMR2|=1<<3; //CH3Ԥװ��ʹ��
	TIM2->CCMR2|=6<<12; //CH4 PWM2ģʽ
	TIM2->CCMR2|=1<<11; //CH4Ԥװ��ʹ��
	TIM2->CCER|=0x1111; //CH1234 ���ʹ��
	TIM2->CR1|=0x01; //ʹ�ܶ�ʱ��2			
	
	TIM2->CCR1=10; //PWMռ�ձ�
	TIM2->CCR2=10;
	TIM2->CCR3=10;
	TIM2->CCR4=10;
	
}

 void TIM4_INT(void)
{    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�õ�2��ַ� 				
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 	   
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ����0-3   ��Ƕ�ף�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //��Ӧ���ȼ�0-3 ���Ŷӣ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

	TIM4->ARR=5000; //�趨�������Զ���װֵ  10000-20ms
	TIM4->PSC=31;   //Ԥ��Ƶ��64���õ�500khz�ļ���ʱ�� 
	TIM4->DIER|=1<<0;   //��������ж�     
	TIM4->DIER|=1<<6;   //�������ж� 
	TIM4->CR1|=0x01;    //ʹ�ܶ�ʱ��4

} 
void TIM3_INT(void)//1ms��ʱ

{ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�õ�2��ַ� 				
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 	   
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ����0-3   ��Ƕ�ף�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //��Ӧ���ȼ�0-3 ���Ŷӣ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	TIM3->ARR=5000; //�趨�������Զ���װֵ  
	TIM3->PSC=31;   //Ԥ��Ƶ��32���õ�1Mhz�ļ���ʱ��  31-1000=1ms 	 
	TIM3->DIER|=1<<0;   //��������ж�     
	TIM3->DIER|=1<<6;   //�������ж� 
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
	
} 


void I2C_Configuration(void) //Ӳ��iic��ʼ��
{
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2 ;
  I2C_InitStructure.I2C_OwnAddress1 =  0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000; 
  I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2C1, &I2C_InitStructure);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void GPIO_Configuration(void) //����ģ��GPIO��ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;//iic�ܽţ�SCL and SDA
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//���ùܽ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

