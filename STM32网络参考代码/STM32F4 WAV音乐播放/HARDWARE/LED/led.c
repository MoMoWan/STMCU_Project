#include <stm32f10x_lib.h>	   
#include "led.h"
		    
	 
//LED IO��ʼ��
void LED_Init(void)
{
	RCC->APB2ENR|=1<<7;    //ʹ��PORTFʱ��

	GPIOF->CRL&=0X00FFFFFF;
	GPIOF->CRL|=0X33000000;//PF6,7,8,9,10�������		   	 		   	 
	GPIOF->CRH&=0XFFFFF000; 
	GPIOF->CRH|=0X00000333; 	 
    GPIOF->ODR|=0x7C0;      //PF6,7,8,9,10 �����											  
}






