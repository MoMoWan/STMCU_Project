#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//Mini STM32������
//LED��������			 
//����ԭ��@ALIENTEK
//2010/5/27

//LED�˿ڶ���
#define LED1 PFout(6)// PF6
#define LED2 PFout(7)// PF7	
#define LED3 PFout(8)// PF8
#define LED4 PFout(9)// PF9
#define LED5 PFout(10)//PF10

void LED_Init(void);//��ʼ��		 				    
#endif

















