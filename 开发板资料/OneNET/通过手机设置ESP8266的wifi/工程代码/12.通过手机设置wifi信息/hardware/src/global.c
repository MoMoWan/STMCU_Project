#include "stm32f10x.h"








void Enable_Interrupt(void) //����ȫ���ж�
{

	

}

void Disable_Interrupt(void) //�ر�ȫ���ж�
{

	

}

void Event_Cmd_On(void) //�����ж�����
{

	TIM_Cmd(TIM6, ENABLE);
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //ʹ�ܽ����ж�

}

void Event_Cmd_Off(void) //�ر��ж�����
{

	TIM_Cmd(TIM6, DISABLE);
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //ʹ�ܽ����ж�

}
