#include "stm32f10x.h"

#include "m6311.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "iwdg.h"

#include <string.h>







void M6311_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	M6311_PWR_ON; //�ϵ�
	DelayXms(50);
	
	//��λģ��	�������λ�Ļ�������������ʱ��6311���޷���ʼ���ɹ���
	M6311_RST_ON;
	DelayXms(100);
	M6311_RST_OFF;
	
	M6311_SendCmd("AT+SSYS?\r\n","OK"); //�л�sim��   0-���ÿ�		1-���ÿ�	����ʹ�����ÿ�
	M6311_SendCmd("AT+SIM1\r\n","OK"); //������ÿ��Ƿ����		����+SIM1: EXSIT
	M6311_SendCmd("AT+CPIN?\r\n", "+CPIN: READY"); //ȷ��SIM��PIN�����������READY����ʾ�����ɹ�
	M6311_SendCmd("AT+CREG?\r\n","0,1"); //ȷ�����������ɹ�,OK		//������0,5��ͬ  �ҵĿ���ʾ0,0  Ȼ����0,1���Ͳ����ˡ� 
	M6311_SendCmd("AT+CSQ\r\n","OK"); //��ѯ�ź�ǿ��,OK
	M6311_SendCmd("AT+CGACT=1,1\r\n","OK"); //����
	M6311_SendCmd("AT+CGATT=1\r\n","OK");
	M6311_SendCmd("AT+CMMUX=0\r\n","OK"); //single way
	M6311_SendCmd("AT+CMMODE=1\r\n","OK"); //����͸��
	M6311_SendCmd("AT+CMTCFG=1,1024,1\r\n","OK"); //����͸������󳤶�2000�ֽڣ������100ms�����ó�hexģʽ
	M6311_SendCmd("AT+IPSTART=\"TCP\",\"183.230.40.39\",876\r\n","CONNECT"); //����ƽ̨
	
	

}

_Bool M6311_SendCmd(char *cmd, char *res) //�����8266��ͬ����ΪҪ�ȴ�����sim�� ��ȡ����ȣ�������û���Ե�����Щָ����Ҫ�ظ��������Ըɴ�ȫ��������
{
	
	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(1)
	{
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(strstr((const char *)usart2Info.usartBuf, res) != NULL)
			{
				Usart2_RcvClr();
				
				return 0;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "ERROR") != NULL)
			{
				Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
				
				continue;
			}
			else
			{
				DelayXms(500);Iwdg_Feed();
				
				Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
			}
		}
		
		DelayXms(10);
		Iwdg_Feed();
	}

}
