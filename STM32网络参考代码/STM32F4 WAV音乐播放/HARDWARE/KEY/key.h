#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//Mini STM32������
//�������� ��������			 
//����ԭ��@ALIENTEK
//2010/5/27 

#define KEY1 PAin(0)    //PA0
#define KEY2 PCin(13)	//PC13	
#define KEY3 PAin(8)	//PA8	 
#define KEY4 PDin(3)	//PD2
	 
void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(void);  //����ɨ�躯��					    
#endif
