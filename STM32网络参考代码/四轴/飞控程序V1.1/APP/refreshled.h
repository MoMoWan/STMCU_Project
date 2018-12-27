#ifndef __REFRESHLED_H__
#define __REFRESHLED_H__

#include "stm32f10x.h"

//0:
//1:
//2:
//3:
extern volatile u8 led_state;//����led״̬


void Led_Refresh_Init(void );//��˸��ʼ��
void TIM6_Config(void);//TIM6��ʼ��

void TIM6_Stop(void);//�ر�TIM6
void TIM6_Start(void);//����TIM6

void Led_Flash0(void);//LED 1������+2�ο���
void Led_Flash1(void);//LED 2�ο���
void Led_Flash2(void);//LED 1������
void Led_Flash3(void);//LED 4�ο���



#endif













