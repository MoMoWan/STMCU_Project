/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	hwtimer.c
	*
	*	���ߣ� 		�ż���
	*
	*	���ڣ� 		2016-11-23
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		��Ƭ����ʱ����ʼ��
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//Э���
#include "onenet.h"

//Ӳ������
#include "hwtimer.h"


/*
************************************************************
*	�������ƣ�	Timer3_4_Init
*
*	�������ܣ�	Timer3��4�Ķ�ʱ����
*
*	��ڲ�����	TIMx��TIM3 ���� TIM4
*				arr������ֵ
*				psc��Ƶֵ
*
*	���ز�����	��
*
*	˵����		timer3��timer4ֻ���и����жϹ���
************************************************************
*/
void Timer3_4_Init(TIM_TypeDef * TIMx, unsigned short arr, unsigned short psc)
{

	TIM_TimeBaseInitTypeDef timer_initstruct;
	NVIC_InitTypeDef nvic_initstruct;
	
	if(TIMx == TIM3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		nvic_initstruct.NVIC_IRQChannel = TIM3_IRQn;
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		nvic_initstruct.NVIC_IRQChannel = TIM4_IRQn;
	}
	
	timer_initstruct.TIM_CounterMode = TIM_CounterMode_Up;
	timer_initstruct.TIM_Period = arr;
	timer_initstruct.TIM_Prescaler = psc;
	
	TIM_TimeBaseInit(TIMx, &timer_initstruct);
	
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); //ʹ�ܸ����ж�
	
	nvic_initstruct.NVIC_IRQChannelCmd = ENABLE;
	nvic_initstruct.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_initstruct.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&nvic_initstruct);
	
	TIM_Cmd(TIMx, ENABLE); //ʹ�ܶ�ʱ��

}

/*
************************************************************
*	�������ƣ�	TIM3_IRQHandler
*
*	�������ܣ�	Timer3�����жϷ�����
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		
		OneNET_CmdHandle();
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}

}

/*
************************************************************
*	�������ƣ�	TIM4_IRQHandler
*
*	�������ܣ�	Timer4�����жϷ�����
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
//void TIM4_IRQHandler(void)
//{

//	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
//	{
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	}

//}
