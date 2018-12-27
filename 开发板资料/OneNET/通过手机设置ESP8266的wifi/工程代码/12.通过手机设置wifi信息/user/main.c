#include "includes.h" //ucos

#include "stm32f10x.h"

#include "led.h"
#include "delay.h"
#include "key.h"
#include "lcd1602.h"
#include "usart.h"
#include "onenet.h"
#include "timer.h"
#include "i2c.h"
#include "gy30.h"
#include "adxl345.h"
#include "sht20.h"
#include "iwdg.h"
#include "stmflash.h"

#include "esp8266.h"
#include "m6311.h"

#include <string.h>



//���Ź�����
#define IWDG_TASK_PRIO		7
#define IWDG_STK_SIZE		64
OS_STK IWDG_TASK_STK[IWDG_STK_SIZE];
void IWDG_Task(void *pdata);

//��������
#define KEY_TASK_PRIO		8
#define KEY_STK_SIZE		512
__align(8) OS_STK KEY_TASK_STK[KEY_STK_SIZE]; //UCOSʹ�ø���������printf��sprintfʱһ��Ҫ8�ֽڶ���
void KEY_Task(void *pdata);

//��������
#define HEART_TASK_PRIO		9
#define HEART_STK_SIZE		256
__align(8) OS_STK HEART_TASK_STK[HEART_STK_SIZE]; //UCOSʹ�ø���������printf��sprintfʱһ��Ҫ8�ֽڶ���
void HEART_Task(void *pdata);

//��������
#define USART_TASK_PRIO		10
#define USART_STK_SIZE		1024
__align(8) OS_STK USART_TASK_STK[USART_STK_SIZE]; //UCOSʹ�ø���������printf��sprintfʱһ��Ҫ8�ֽڶ���
void USART_Task(void *pdata);

//����������
#define SENSOR_TASK_PRIO	11
#define SENSOR_STK_SIZE		512
__align(8) OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE]; //UCOSʹ�ø���������printf��sprintfʱһ��Ҫ8�ֽڶ���
void SENSOR_Task(void *pdata);

//AP������������
#define APMODE_TASK_PRIO	6 //���ȼ�Ҫ�Ƚϸ߲��У�������Ϊ12��ʱ�򣬻���һ�����ʳ������Ӳ����ֻ�server��
#define APMODE_STK_SIZE		512
OS_STK APMODE_TASK_STK[APMODE_STK_SIZE]; //
void APMODE_Task(void *pdata);






#define WIFI_GPRS	1 //1-wifi		0-gprs




void Hardware_Init(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Delay_Init();
	
	Led_Init();
	
	Key_Init();
	
	Lcd1602_Init();
	
	//Usart1_Init(115200); //��ʼ������   115200bps
	Usart2_Init(115200); //��ʼ������   115200bps
	
	IIC_Init();
	
	ADXL345_Init();
	
	GY30_Init();
	
	Iwdg_Init(4, 1250); //64��Ƶ��ÿ��625�Σ�����1250�Σ�2s
	
#if(WIFI_GPRS == 1)

	if(Flash_NeedErase()) //���������
	{
		memset(esp8266Info.staName, 0, sizeof(esp8266Info.staName)); //���֮ǰ������
		Flash_Read(SSID_ADDRESS, esp8266Info.staName, sizeof(esp8266Info.staName)); //����ssid
		Lcd1602_DisString(0x80, "%s", esp8266Info.staName); //��ʾ
		DelayXms(10); //��ʱ��Ҳ���Ǳ����
		
		memset(esp8266Info.staPass, 0, sizeof(esp8266Info.staPass)); //���֮ǰ������
		Flash_Read(PSWD_ADDRESS, esp8266Info.staPass, sizeof(esp8266Info.staPass)); //����password
		Lcd1602_DisString(0xC0, "%s", esp8266Info.staPass); //��ʾ

		DelayMs(1500); //��ʱ��ʾ
		Iwdg_Feed();
	}
	else //û������
	{
		Lcd1602_DisString(0x80, "No Wifi Info in Flash");
		DelayMs(1500); //��ʱ��ʾ
		Iwdg_Feed();
	}
	
	ESP8266_Mode(); //�����������ʹ��sta client����·�ɺ�ƽ̨����������κ�һ������ͻ�ʹ��ap client(�����ֻ�������ߵ�wifi����ap��
					//Ȼ��������ͨѶ���������˿ڣ��ȴ�����)�����ӵ��ֻ�APP�ϣ�������ʾ�����Ӧ����Ϣ��
	
#elif(WIFI_GPRS == 0)
	
	Lcd1602_DisString(0x80, "Wait M6311...   ");
	
	M6311_Init();
	esp8266Info.netWork = 1; //������ģʽ
	
	Lcd1602_DisString(0x80, "M6311 OK        ");
	
	DelayMs(1500);
	
#endif
	
	if(esp8266Info.netWork) //�����apģʽ���Ͳ���������ƽ̨������
		OneNet_DevLink(DEVICEID, APIKEY);
	
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) //����ǿ��Ź���λ����ʾ
	{
		RCC_ClearFlag();
		
		Lcd1602_DisString(0x8F, "1");
	}
	
	Timer1_8_Init(TIM8, 300, 3599);
	
	UCOS_TimerInit();

}

int main(void)
{
	
	Hardware_Init();

	OSInit();
	
	OSTaskCreate(IWDG_Task, (void *)0, (OS_STK*)&IWDG_TASK_STK[IWDG_STK_SIZE - 1], IWDG_TASK_PRIO);
	
	OSTaskCreate(KEY_Task, (void *)0, (OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE - 1], KEY_TASK_PRIO);
	
	OSTaskCreate(HEART_Task, (void *)0, (OS_STK*)&HEART_TASK_STK[HEART_STK_SIZE - 1], HEART_TASK_PRIO);
	
	OSTaskCreate(USART_Task, (void *)0, (OS_STK*)&USART_TASK_STK[USART_STK_SIZE - 1], USART_TASK_PRIO);
	
	OSTaskCreate(SENSOR_Task, (void *)0, (OS_STK*)&SENSOR_TASK_STK[SENSOR_STK_SIZE - 1], SENSOR_TASK_PRIO);
	
	OSTaskCreate(APMODE_Task, (void *)0, (OS_STK*)&APMODE_TASK_STK[APMODE_STK_SIZE - 1], APMODE_TASK_PRIO);
	
	OSStart();

}

void IWDG_Task(void *pdata)
{

	while(1)
	{
	
		Iwdg_Feed();
		
		OSTimeDly(50);
	
	}

}

void KEY_Task(void *pdata)
{

	while(1)
	{
	
		switch(Keyboard())
		{
			case KEY0DOWN:
			
				++ledStatus.LedRedSta;
				ledStatus.LedRedSta %= 300;
				TIM_SetCompare2(TIM8, ledStatus.LedRedSta);
			
			break;
			
			case KEY1DOWN:
				
				++ledStatus.LedGreenSta;
				ledStatus.LedGreenSta %= 300;
				TIM_SetCompare3(TIM8, ledStatus.LedGreenSta);
			
			break;
				
			case KEY2DOWN:
				
				if(ledStatus.LedBlueSta == LED_ON)
				{
					ledStatus.LedBlueSta = LED_OFF;
					Led_Blue_Set(LED_OFF);
				}
				else
				{
					ledStatus.LedBlueSta = LED_ON;
					Led_Blue_Set(LED_ON);
				}
				
				if(esp8266Info.netWork) //ֻ�����ӵ����������������ݷ���
					OneNet_SendData();
			
			break;
			
			case KEY3DOWN:
				
				if(ledStatus.LedYellowSta == LED_ON)
				{
					ledStatus.LedYellowSta = LED_OFF;
					Led_Yellow_Set(LED_OFF);
				}
				else
				{
					ledStatus.LedYellowSta = LED_ON;
					Led_Yellow_Set(LED_ON);
				}
				
				if(esp8266Info.netWork) //ֻ�����ӵ����������������ݷ���
					OneNet_SendData();
			
			break;
		}
		
		OSTimeDly(10);
	
	}

}

void HEART_Task(void *pdata)
{

	while(1)
	{
	
		if(esp8266Info.netWork) //ֻ�����ӵ����������������������ݼ������
		{
			OSTimeDlyHMSM(0, 0, 30, 0);
			OneNet_SendData();
			
			OSTimeDlyHMSM(0, 0, 30, 0);
			HeartBeat(&usart2Info);
		}
		else //��ʹ��APģʽʱ������������������
		{
			OSTimeDly(20);
		}
	
	}

}

void USART_Task(void *pdata)
{

	while(1)
	{
	
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(esp8266Info.netWork) //�����staģʽ�����յ����������ڿ���֮���
			{
				EDPKitCmd(&usart2Info);
				
				OneNetApp(&usart2Info);
			}
			else //�����apģʽ
			{
				if(esp8266Info.ssidOK == 1) //���յ�ssid
				{
					memset(esp8266Info.staName, 0, sizeof(esp8266Info.staName)); //���֮ǰ������
					memcpy(esp8266Info.staName, usart2Info.usartBuf, sizeof(esp8266Info.staName));
					
					//д��ǰ�߲���Ƭ��
					FLASH_Unlock();	//����	����������ܲ���
					FLASH_ErasePage(SSID_ADDRESS); //��������ҳ
					FLASH_Lock(); //����
					
					//�������
					Flash_Write(SSID_ADDRESS, esp8266Info.staName, strlen(esp8266Info.staName)); //д��ssid
					
					esp8266Info.ssidOK = 2; //���ssid���õڶ���
				}
				else if(esp8266Info.pswdOK == 1) //���յ�pswd
				{
					memset(esp8266Info.staPass, 0, sizeof(esp8266Info.staPass)); //���֮ǰ������
					memcpy(esp8266Info.staPass, usart2Info.usartBuf, sizeof(esp8266Info.staPass));
					//�������
					Flash_Write(PSWD_ADDRESS, esp8266Info.staPass, strlen(esp8266Info.staPass)); //д��password
					
					esp8266Info.pswdOK = 2; //���pswd���õڶ���
				}
			}
			
			Usart2_RcvClr();
		}
		
		OSTimeDly(2);
	
	}

}

void SENSOR_Task(void *pdata) //
{

	while(1)
	{
		
		if(esp8266Info.netWork)
		{
			TIM_Cmd(TIM6, DISABLE);
		
			ADXL345_GetValue();Iwdg_Feed();
			GY30_GetValue();Iwdg_Feed();
			SHT20_GetValue();Iwdg_Feed();
			
			Lcd1602_DisString(0x80, "X%0.1f,Y%0.1f,Z%0.1f", adxlInfo.incidence_Xf, adxlInfo.incidence_Yf, adxlInfo.incidence_Zf);
			Lcd1602_DisString(0xC0, "%0.1fC,%0.1f%%,%dLX", sht20Info.tempreture, sht20Info.humidity, gy30Info.lightVal);
			
			TIM_Cmd(TIM6, ENABLE);
		}
		
		OSTimeDly(100);
	
	}

}

void APMODE_Task(void *pdata)
{

	while(1)
	{
	
		if(esp8266Info.netWork == 0) //��ʹ��APģʽʱ������������������
		{
			if(esp8266Info.ssidOK == 0)
			{
				UsartPrintf(USART2, "������Wifi����\r\n"); //��ѡ����ʾ
				
				Lcd1602_Clear(0x80); //�����һ����ʾ����
				Lcd1602_DisString(0x80, "Enter SSID"); //tip
				
				esp8266Info.ssidOK = 1; //���ssid���õ�һ��
			}
			else if(esp8266Info.ssidOK == 2)
			{
				UsartPrintf(USART2, "Wifi����: %s\r\n", esp8266Info.staName); //��ѡ����ʾ
				
				Lcd1602_Clear(0x80); //�����һ����ʾ����
				Lcd1602_DisString(0x80, "%s", esp8266Info.staName); //��ʾ�����ssid
				
				esp8266Info.ssidOK = 3; //���ssid�������һ��
			}
			
			else if(esp8266Info.pswdOK == 0 && esp8266Info.ssidOK == 3) //���ssid������ϣ���ʼpswd������
			{
				UsartPrintf(USART2, "������Wifi����\r\n"); //��ѡ����ʾ
				
				Lcd1602_Clear(0xC0); //����ڶ�����ʾ����
				Lcd1602_DisString(0xC0, "Enter PassWord"); //tip
				
				esp8266Info.pswdOK = 1; //���pswd���õ�һ��
			}
			else if(esp8266Info.pswdOK == 2 && esp8266Info.ssidOK == 3)
			{
				UsartPrintf(USART2, "Wifi����: %s\r\n", esp8266Info.staPass); //��ѡ����ʾ
				
				Lcd1602_Clear(0xC0); //����ڶ�����ʾ����
				Lcd1602_DisString(0xC0, "%s", esp8266Info.staPass); //tip
				
				esp8266Info.pswdOK = 3; //���pswd�������һ��
				
				OSTimeDlyHMSM(0, 0, 5, 0); //�л�����5s���ң����ﵱ��ʾʹ��
				Lcd1602_Clear(0xFF); //���������ʾ����
			}
			
			if(esp8266Info.pswdOK == 3 && esp8266Info.ssidOK == 3) //��ssid��pswd���������
			{
				TIM_Cmd(TIM2, DISABLE); //�رն�ʱ��
				
				ESP8266_Mode(); //�Զ�ѡ��ģʽ��119��������
				
				if(esp8266Info.netWork) //��staģʽ�ˣ���������˻�������������ƽ̨
					OneNet_DevLink(DEVICEID, APIKEY);
				else //�������APģʽ���������´����ssid��password���Ǵ�ģ���Ҫ���������������ʼ��
				{
					esp8266Info.ssidOK = 0; //������������
					esp8266Info.pswdOK = 0; //������������
				}
				
				TIM_Cmd(TIM2, ENABLE);
			}
		}
		
		OSTimeDly(20);
	
	}

}
