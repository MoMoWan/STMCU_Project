/***********************************************

����: timer.c
����: ��������
��ַ��http://qiuyangdz.taobao.com
����: 2014/05/18
�汾��v1.0
����: ��ʱ���������жϳ�ʼ��
˵����timer2����ĳ�ʼ��
*************************************************/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "timer.h"
/*************************************************

���ƣ�timer_init(void)
���ܣ�timer3�����ʼ�����ж� ��ʱʱ�䣩
�����������
�����������
����ֵ��  ��
**************************************************/
void timer_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 7200 - 1;    
  TIM_TimeBaseStructure.TIM_Prescaler = 25 - 1;  // (Period + 1) * (Prescaler + 1) / 72M = 0.05s
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM3 enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}
/***************************END OF FILE**********************************************************************/
