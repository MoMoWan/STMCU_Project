#include "refreshled.h"
#include "led.h"




volatile u8 led_state;//����led״̬
u32 time6_tick;//Time6������

void Led_Refresh_Init(void )//��˸��ʼ��
{
	TIM6_Config();
	time6_tick = 0;
	
	led_state = 0xFF;
}
/*
 * ��������TIM6_Config
 * ����  ��TIM6���� NVIC�ж����� 10ms�ж�һ��
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//����NVIC��TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;//ͨ������ΪTIM6
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;//��ռ3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//��TIM6�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//д������
	
	//����TIM6
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//����TIM6ʱ��
  //TIM_DeInit(TIM6);//TIM6��ʼ��Ϊȱʡֵ

	TIM_TimeBaseStructure.TIM_Period=10000;//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);//ʱ��Ԥ��Ƶ�� 72M/72
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//������Ƶ TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
	
  TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);//����TIM6
    
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);//�������жϱ�־
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//��������ж�
	TIM_Cmd(TIM6,ENABLE);//����TIM6����
		
}


/*
 * ��������TIM6_Start
 * ����  ������TIM6
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM6_Start(void)
{
	time6_tick = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
}

/*
 * ��������TIM6_Stop
 * ����  ���ر�TIM2
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����	
 */
void TIM6_Stop(void)
{
	TIM_Cmd(TIM6,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,DISABLE);
}


//*****TIM6�жϺ���---10ms---*****//
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)//���TIM6����ж��Ƿ���
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		
		switch(led_state)
		{
				case 0:Led_Flash0();break;
				case 1:Led_Flash1();break;
				case 2:Led_Flash2();break;
				case 3:Led_Flash3();break;
		}
		time6_tick++;
		time6_tick = time6_tick%200;//2sһ������
	}		
}



void Led_Flash0(void) //LED 1������+2�ο���
{
	if(time6_tick == 0)
	{
		LED0(ON);
	}
	else if(time6_tick == 15)
	{
		LED0(OFF);
	}
		
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
}

void Led_Flash1(void) //LED 2�ο���
{
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
}

void Led_Flash2(void) //LED 1������
{
	if(time6_tick == 0)
	{
		LED0(ON);
	}
	else if(time6_tick == 15)
	{
		LED0(OFF);
	}
}


void Led_Flash3(void) //LED 4�ο���
{
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
	else if(time6_tick == 120)
	{
		LED0(ON);
	}
	else if(time6_tick == 125)
	{
		LED0(OFF);
	}
	else if(time6_tick == 130)
	{
		LED0(ON);
	}
	else if(time6_tick == 135)
	{
		LED0(OFF);
	}
}











