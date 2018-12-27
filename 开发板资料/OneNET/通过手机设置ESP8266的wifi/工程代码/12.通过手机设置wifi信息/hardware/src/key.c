#include "stm32f10x.h"

#include "key.h"
#include "delay.h"
#include "led.h"







//PC13--SW5
//PC12--SW4
//PC11--SW3
//PD2--SW2

void Key_Init(void)
{

	GPIO_InitTypeDef gpioInitStructure;
	EXTI_InitTypeDef extiInitStructure;
	NVIC_InitTypeDef nvicInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	//IO����
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
	GPIO_Init(GPIOC, &gpioInitStructure);
	
	//IO����
	gpioInitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &gpioInitStructure);

#if(KEY_INT == 1)
	
	//�ж�����		PC11
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
	extiInitStructure.EXTI_Line = EXTI_Line11;
	extiInitStructure.EXTI_LineCmd = ENABLE;
	extiInitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	extiInitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	//�жϳ�ʼ��
	EXTI_Init(&extiInitStructure);
	
	//�ж�����		PC12
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
	extiInitStructure.EXTI_Line = EXTI_Line12;
	//�жϳ�ʼ��
	EXTI_Init(&extiInitStructure);
	
	//�ж�����		PC13
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	extiInitStructure.EXTI_Line = EXTI_Line13;
	//�жϳ�ʼ��
	EXTI_Init(&extiInitStructure);
	
	//�ж�����		PD2
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
	extiInitStructure.EXTI_Line = EXTI_Line2;
	//�жϳ�ʼ��
	EXTI_Init(&extiInitStructure);
	
	nvicInitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvicInitStructure);
	
	nvicInitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStructure);
	
#endif
	

}

unsigned char KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM)
{
	
	if(GPIOX == GPIOC)
	{
		if(!GPIO_ReadInputDataBit(GPIOC, NUM)) //����  Ϊ��
		{
			return KEYDOWN;
		}
		else					   //����  Ϊ��
		{
			return KEYUP;
		}
	}
	else if(GPIOX == GPIOD)
	{
		if(!GPIO_ReadInputDataBit(GPIOD, NUM)) //����  Ϊ��
		{
			return KEYDOWN;
		}
		else					 //����  Ϊ��
		{
			return KEYUP;
		}
	}
	
	return KEYUP;
	
}

unsigned char Keyboard(void)
{
	
	static unsigned char keyStatus = KEYNONE;
	static unsigned int keyBusyFlag = 0;
	
	if(KeyScan(GPIOC, KEY0) == KEYDOWN && !(keyBusyFlag & (~(1 << 0)))) //������� ����������δ����
	{
		if(keyStatus == KEY0DOWN) //���һֱ����
		{
			return KEY0DOWNREPEAT; //����һֱ����
		}
		
		keyBusyFlag |= 1 << 0;//�˰�������æ״̬
		keyStatus = KEY0DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY0) == KEYUP && keyStatus == KEY0DOWN) //����ͷ�
	{
		keyBusyFlag &= ~(1 << 0);//�˰������ڿ���״̬
		keyStatus = KEY0UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOC, KEY1) == KEYDOWN && !(keyBusyFlag & (~(1 << 1)))) //������� ����������δ����
	{
		if(keyStatus == KEY1DOWN) //���һֱ����
		{
			return KEY1DOWNREPEAT; //����һֱ����
		}
		
		keyBusyFlag |= 1 << 1;
		keyStatus = KEY1DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY1) == KEYUP && keyStatus == KEY1DOWN) //����ͷ�
	{
		keyBusyFlag &= ~(1 << 1);
		keyStatus = KEY1UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOC, KEY2) == KEYDOWN && !(keyBusyFlag & (~(1 << 2)))) //������� ����������δ����
	{
		if(keyStatus == KEY2DOWN) //���һֱ����
		{
			return KEY2DOWNREPEAT; //����һֱ����
		}
		
		keyBusyFlag |= 1 << 2;
		keyStatus = KEY2DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY2) == KEYUP && keyStatus == KEY2DOWN) //����ͷ�
	{
		keyBusyFlag &= ~(1 << 2);
		keyStatus = KEY2UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOD, KEY3) == KEYDOWN && !(keyBusyFlag & (~(1 << 3)))) //������� ����������δ����
	{
		if(keyStatus == KEY3DOWN) //���һֱ����
		{
			return KEY3DOWNREPEAT; //����һֱ����
		}
		
		keyBusyFlag |= 1 << 3;
		keyStatus = KEY3DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOD, KEY3) == KEYUP && keyStatus == KEY3DOWN) //����ͷ�
	{
		keyBusyFlag &= ~(1 << 3);
		keyStatus = KEY3UP;
		return keyStatus;
	}
	
	keyBusyFlag = 0;
	keyStatus = KEYNONE;
	return keyStatus;
	
}

void EXTI15_10_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line11) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) == 0)
		{
			//do something ...
			if(Led_Red_Get() == 1)
				Led_Red_Set(LED_OFF);
			else
				Led_Red_Set(LED_ON);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line12) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0)
		{
			//do something ...
			if(Led_Green_Get() == 1)
				Led_Green_Set(LED_OFF);
			else
				Led_Green_Set(LED_ON);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line13) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0)
		{
			//do something ...
//			if(Led_Blue_Get() == 1)
//				Led_Blue_Set(LED_OFF);
//			else
//				Led_Blue_Set(LED_ON);
		}
	}
	
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);

}

void EXTI2_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 0)
		{
			//do something ...
			if(Led_Yellow_Get() == 1)
				Led_Yellow_Set(LED_OFF);
			else
				Led_Yellow_Set(LED_ON);
		}
	}
	
	EXTI_ClearITPendingBit(EXTI_Line2);

}
