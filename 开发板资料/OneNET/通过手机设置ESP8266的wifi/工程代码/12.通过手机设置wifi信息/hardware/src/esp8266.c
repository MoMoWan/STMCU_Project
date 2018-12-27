#include "stm32f10x.h"

#include "esp8266.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "lcd1602.h"
#include "iwdg.h"

#include <string.h>






ESP8266_INFO esp8266Info = {"", "", "NULL", "NULL", "876", "192.168.4.2", "", "OneNET", "12345678", "8086", 0, 0, 0, 0, 0}; //netWork ssid pswd ����


//reset		PC4
//USART2

void ESP8266_QuitTrans(void) //�˳�͸��ģʽ
{

	while((USART2->SR & 0X40) == 0);	//�ȴ����Ϳ�
	USART2->DR = '+';      
	DelayXms(15);					//���ڴ�����֡ʱ��(10ms)
	
	while((USART2->SR & 0X40) == 0);	//�ȴ����Ϳ�
	USART2->DR = '+';        
	DelayXms(15);					//���ڴ�����֡ʱ��(10ms)
	
	while((USART2->SR & 0X40) == 0);	//�ȴ����Ϳ�
	USART2->DR = '+';        
	DelayXms(500);					//�ȴ�500ms
	Iwdg_Feed();
	
	ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK"); //�ر�͸��ģʽ

}

void ESP8266_ApInit(void)
{
	
	char cfgBuffer[70];
	
	ESP8266_QuitTrans();
	DelayMs(10);
	
	ESP8266_SendCmd("AT\r\n", "OK");
	
	ESP8266_SendCmd("AT+CWMODE=2\r\n", "OK");
	
	ESP8266_SendCmd("AT+RST\r\n", "OK");
	
	DelayMs(500);Iwdg_Feed();
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CWSAP=\"");
	strcat(cfgBuffer, esp8266Info.apName);
	strcat(cfgBuffer, "\",\"");
	strcat(cfgBuffer, esp8266Info.apPass);
	strcat(cfgBuffer, "\",1,4\r\n\"");
	ESP8266_SendCmd(cfgBuffer, "OK");
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CIPSTART=\"TCP\",\"");
	strcat(cfgBuffer, esp8266Info.apip);
	strcat(cfgBuffer, "\",");
	strcat(cfgBuffer, esp8266Info.apPort);
	strcat(cfgBuffer, "\r\n");
	while(ESP8266_SendCmd(cfgBuffer, "OK"))
	{
		Led_Blue_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Blue_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
	}
	
	ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+CIPSEND\r\n", ">");
	
	esp8266Info.netWork = 0; //����ģʽ

}

unsigned char ESP8266_StaInit(void)
{
	
	unsigned char errCount = 0;
	char cfgBuffer[70];
	
	ESP8266_QuitTrans();
	DelayMs(10);
	
	ESP8266_SendCmd("AT\r\n", "OK");
	
	ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+RST\r\n", "OK");
	
	DelayMs(500);Iwdg_Feed();
	
	ESP8266_SendCmd("AT+CIFSR\r\n", "OK");
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CWJAP=\"");
	strcat(cfgBuffer, esp8266Info.staName);
	strcat(cfgBuffer, "\",\"");
	strcat(cfgBuffer, esp8266Info.staPass);
	strcat(cfgBuffer, "\"\r\n\"");
	while(ESP8266_SendCmd(cfgBuffer, "OK"))
	{
		Led_Blue_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Blue_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
		
		if(++errCount >= 5)
			return 1; //����wifi��Ϣ����
	}
	
	while(ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n", "OK"))
	{
		Led_Yellow_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Yellow_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
		
		if(++errCount >= 5)
			return 2; //����ƽ̨��Ϣ����
	}
	
	ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+CIPSEND\r\n", ">");
	
	esp8266Info.netWork = 1; //������ģʽ
	
	return ESP_OK;

}

_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	
	unsigned int timeOut = 200;
	
	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(timeOut--)
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
		}
		
		DelayXms(10);
		Iwdg_Feed();
	}
	
	return 1;

}

void ESP8266_Mode(void)
{

	Lcd1602_Clear(0xFF); //���������ʾ����
	Lcd1602_DisString(0x80, "Wait ESP8266 STA"); //��ʾ���Ƚ���staģʽ���ӵ�·��
	
	esp8266Info.err = ESP8266_StaInit(); //��ȡ���ӽ��	0-�ɹ�
	
	if(esp8266Info.err == 1) //���·����Ϣ����������apģʽ ͨ���ֻ���ȡssid��password
	{
		Lcd1602_Clear(0xFF); //���������ʾ����
		Lcd1602_DisString(0x80, "Wifi info Error"); //�����˵wifi��ssid��pswd���ô���
		Lcd1602_DisString(0xC0, "Use APP -> 8266"); //�����ֻ�������ߵ�wifi����ap��Ȼ��������ͨѶ���������˿ڣ��ȴ�����
		
		ESP8266_ApInit(); //ʹ��apģʽ
		
		Lcd1602_Clear(0xFF); //���������ʾ����
		Lcd1602_DisString(0x80, "ESP8266 AP OK   "); //��ʾap ok
	}
	else if(esp8266Info.err == 2) //�����ƽ̨��Ϣ����û�в��Ի᲻���Ҳû������������ƽ̨ip��port�Ĺ��ܣ���������Բ�����
	{
		Lcd1602_Clear(0xFF);
		Lcd1602_DisString(0x80, "PT info Error");
		Lcd1602_DisString(0xC0, "Use APP -> 8266");
		
		ESP8266_ApInit();
		
		Lcd1602_Clear(0xFF);
		Lcd1602_DisString(0x80, "ESP8266 AP OK   ");
	}
	else
		Lcd1602_DisString(0x80, "ESP8266 STA OK  "); //�������ʹ��staģʽok��
	
	DelayMs(1500); //��ʱ2s
	Iwdg_Feed();

}

_Bool ESP8266GetStatus(void)
{
	
	_Bool status = 1;

	Usart_SendString(USART2, "AT+CIPSTATUS\r\n",  14);
	
	while(1)
	{
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(strstr((const char *)usart2Info.usartBuf, "STATUS:2")) //���IP
			{
				status = 1;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:3")) //��������
			{
				status = 0;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:4")) //ʧȥ����
			{
				status = 1;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:5")) //�������
			{
				status = 1;
			}
			
			memset(usart2Info.usartBuf, 0, sizeof(usart2Info.usartBuf));
			
			break;
		}
	}
	
	return status;

}
