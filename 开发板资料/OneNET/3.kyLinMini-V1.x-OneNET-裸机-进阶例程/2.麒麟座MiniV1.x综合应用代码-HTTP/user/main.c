/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	main.c
	*
	*	���ߣ� 		�ż���
	*
	*	���ڣ� 		2017-01-011
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		����onenet���ϴ����ݺ��������
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//���
#include "framework.h"

//����Э���
#include "onenet.h"
#include "fault.h"

//�����豸
#include "net_device.h"

//Ӳ������
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "hwtimer.h"
#include "i2c.h"
#include "sht20.h"
#include "at24c02.h"
#include "selfcheck.h"
#include "rtc.h"

//����������
#include "dataStreamName.h"

//C��
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


#define NET_COUNT	7			//�������

#define NET_TIME	60			//�趨ʱ��--��λ��

unsigned short timerCount = 0;	//ʱ�����--��λ��


char myTime[24];


//������
DATA_STREAM dataStream[] = {
								{ZW_REDLED, &led_status.Led5Sta, TYPE_BOOL, 1},
								{ZW_GREENLED, &led_status.Led4Sta, TYPE_BOOL, 1},
								{ZW_YELLOWLED, &led_status.Led3Sta, TYPE_BOOL, 1},
								{ZW_BLUELED, &led_status.Led2Sta, TYPE_BOOL, 1},
								{ZW_TEMPERATURE, &sht20_info.tempreture, TYPE_FLOAT, 1},
								{ZW_HUMIDITY, &sht20_info.humidity, TYPE_FLOAT, 1},
								{ZW_TIME, myTime, TYPE_STRING, 1},
								{"GPS", &gps, TYPE_GPS, 0},
								{ZW_ERRTYPE, &net_fault_info.net_fault_level_r, TYPE_UCHAR, 1},
							};
unsigned char dataStreamCnt = sizeof(dataStream) / sizeof(dataStream[0]);


/*
************************************************************
*	�������ƣ�	Hardware_Init
*
*	�������ܣ�	Ӳ����ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		��ʼ����Ƭ�������Լ�����豸
************************************************************
*/
void Hardware_Init(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);								//�жϿ�������������

	Delay_Init();																//systick��ʼ��
	
	Usart1_Init(115200); 														//��ʼ������   115200bps
#if(USART_DMA_RX_EN)
	USARTx_ResetMemoryBaseAddr(USART_DEBUG, (unsigned int)alterInfo.alterBuf, sizeof(alterInfo.alterBuf), USART_RX_TYPE);
#endif
	
	Led_Init();																	//LED��ʼ��
	
	IIC_Init();																	//���IIC���߳�ʼ��
	
	RTC_Init();																	//��ʼ��RTC
	
	Check_PowerOn(); 															//�ϵ��Լ�

	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) 								//����ǿ��Ź���λ����ʾ
	{
		UsartPrintf(USART_DEBUG, "WARN:	IWDG Reboot\r\n");
		
		RCC_ClearFlag();														//������Ź���λ��־λ
		
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r
														= NET_FAULT_LEVEL_5;	//����ȼ�5
		
		net_device_info.reboot = 1;
	}
	else
	{
		UsartPrintf(USART_DEBUG, "2.DEVID: %s,     APIKEY: %s\r\n"
								, onenet_info.devID, onenet_info.apiKey);
		
		net_device_info.reboot = 0;
	}
	
	//Iwdg_Init(4, 1250); 														//64��Ƶ��ÿ��625�Σ�����1250�Σ�2s
	
	Timer3_4_Init(TIM3, 49, 35999);												//72MHz��36000��Ƶ-500us��50����ֵ�����ж�����Ϊ500us * 50 = 25ms
	
	UsartPrintf(USART_DEBUG, "3.Hardware init OK\r\n");							//��ʾ��ʼ�����

}

/*
************************************************************
*	�������ƣ�	USART_Task
*
*	�������ܣ�	����ƽ̨�·�������
*
*	��ڲ�����	void���͵Ĳ���ָ��
*
*	���ز�����	��
*
*	˵����		���ڽ�������������ģʽ��ʱ���ȴ�ƽ̨�·����������������
************************************************************
*/
void USART_Task(void)
{
	
	if(onenet_info.cmd_ptr)
	{
		OneNet_RevPro(onenet_info.cmd_ptr);
		
		onenet_info.cmd_ptr = NULL;
	}

}

/*
************************************************************
*	�������ƣ�	STATUS_Task
*
*	�������ܣ�	״̬���
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void STATUS_Task(void)
{

	OneNet_Status();			//״̬���

}

/*
************************************************************
*	�������ƣ�	SEND_Task
*
*	�������ܣ�	�ϴ�����������
*
*	��ڲ�����	void���͵Ĳ���ָ��
*
*	���ز�����	��
*
*	˵����		���ݷ�������
************************************************************
*/
void SEND_Task(void)
{

	onenet_info.sendData = SEND_TYPE_DATA;		//�ϴ����ݵ�ƽ̨

}

/*
************************************************************
*	�������ƣ�	SENSOR_Task
*
*	�������ܣ�	���������ݲɼ�����ʾ
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		���������ݲɼ����񡣽�����Ӵ����������ݲɼ�����ȡ����ʾ
************************************************************
*/
void SENSOR_Task(void)
{
	
	if(check_info.SHT20_OK == DEV_OK) 									//ֻ���豸����ʱ���Ż��ȡֵ����ʾ
	{
		SHT20_GetValue();												//�ɼ�����������
	}

}

/*
************************************************************
*	�������ƣ�	DATA_Task
*
*	�������ܣ�	���ݷ���������
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void DATA_Task(void)
{
	
	switch(onenet_info.sendData)
	{
		case SEND_TYPE_DATA:
			
			onenet_info.sendData = OneNet_SendData(FORMAT_TYPE3, onenet_info.devID, onenet_info.apiKey, dataStream, dataStreamCnt);//�ϴ����ݵ�ƽ̨

		break;
	}

}

/*
************************************************************
*	�������ƣ�	DATALIST_Task
*
*	�������ܣ�	ѭ������������ߴ����͵����ݿ�
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void DATALIST_Task(void)
{

	if(NET_DEVICE_CheckListHead())
	{
		NET_DEVICE_SendData(NET_DEVICE_GetListHeadBuf(), NET_DEVICE_GetListHeadLen());
		NET_DEVICE_DeleteDataSendList();
	}

}

/*
************************************************************
*	�������ƣ�	FAULT_Task
*
*	�������ܣ�	����״̬������
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		���ϴ������񡣵�������������豸����ʱ�����Ƕ�Ӧ��־λ��Ȼ���н��д���
************************************************************
*/
void FAULT_Task(void)
{

	if(net_fault_info.net_fault_level != NET_FAULT_LEVEL_0)					//��������־������
	{
		UsartPrintf(USART_DEBUG, "WARN:	NET Fault Process\r\n");
		
		NET_Fault_Process();												//�����������
	}

}

/*
************************************************************
*	�������ƣ�	NET_Task
*
*	�������ܣ�	�������ӡ�ƽ̨����
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		���������������񡣻������������߼����������״̬������д�����״̬��Ȼ���������������
************************************************************
*/
void NET_Task(void)
{

	if(onenet_info.netWork == 0)
	{
		if(!onenet_info.netWork && (check_info.NET_DEVICE_OK == DEV_OK))		//��û������ �� ����ģ���⵽ʱ
		{
			if(!NET_DEVICE_Init(onenet_info.protocol, onenet_info.ip, onenet_info.port))//��ʼ�������豸������������
			{
				onenet_info.netWork = 1;

				if(gps.flag)
					dataStream[7].flag = 1;									//GPS������׼���ϴ�
			}
		}
		if(check_info.NET_DEVICE_OK == DEV_ERR) 								//�������豸δ�����
		{
			if(!NET_DEVICE_Exist())											//�����豸���
			{
				UsartPrintf(USART_DEBUG, "NET Device :Ok\r\n");
				check_info.NET_DEVICE_OK = DEV_OK;							//��⵽�����豸�����
			}
			else
				UsartPrintf(USART_DEBUG, "NET Device :Error\r\n");
		}
	}

}

/*
************************************************************
*	�������ƣ�	CLOCK_Task
*
*	�������ܣ�	����Уʱ��ʱ����ʾ
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void CLOCK_Task(void)
{
	
#if(NET_TIME_EN == 1)
	static unsigned int second = 0, second_pre = 0, err_count = 0;	//second��ʵʱʱ�䣬second_pre��ֵ�Ƚϡ�err_count��ȡ��ʱ
	static struct tm *time;
	static _Bool get_net_time = 1;
#endif

#if(NET_TIME_EN == 1)
	if(get_net_time)												//��Ҫ��ȡʱ��
	{
		dataStream[6].flag = 0;										//���ϴ�ʱ��
		
		if(FW_GetTicks() - err_count >= 24000)						//ʮ���ӻ���ȡ���������»�ȡ(25msһ��)
		{
			err_count = 0;
			net_device_info.net_time = 0;
			onenet_info.netWork = 0;
			NET_DEVICE_ReConfig(0);
		}
		
		if(net_device_info.net_time)
		{
			second = RTC_GetCounter();
			
			if(((net_device_info.net_time <= second + 300) && (net_device_info.net_time >= second - 300)) || (second <= 100))
			{														//����ڡ�5�����ڣ�����Ϊʱ����ȷ
				RTC_SetTime(net_device_info.net_time + 4);			//����RTCʱ�䣬��4�ǲ��ϴ�ŵ�ʱ���
				
				get_net_time = 0;
				err_count = 0;
				
				dataStream[6].flag = 1;								//�ϴ�ʱ��
			}
		}
	}
	
	second = RTC_GetCounter();										//��ȡ��ֵ
	
	if(second > second_pre)
	{
		second_pre = second;
		time = localtime((const time_t *)&second);					//����ֵתΪtm�ṹ����ʾ��ʱ��
		
		memset(myTime, 0, sizeof(myTime));
		snprintf(myTime, sizeof(myTime), "%d-%d-%d %d:%d:%d",
						time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
						time->tm_hour, time->tm_min, time->tm_sec);
	
		if(time->tm_hour == 0 && time->tm_min == 0 && time->tm_sec == 0)//ÿ��0��ʱ������һ��ʱ��
		{
			get_net_time = 1;
			err_count = FW_GetTicks();
			net_device_info.net_time = 0;
			onenet_info.netWork = 0;
			NET_DEVICE_ReConfig(0);
		}
	}
#endif

}

/*
************************************************************
*	�������ƣ�	NET_Timer
*
*	�������ܣ�	��ʱ�������״̬��־λ
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		��ʱ�����񡣶�ʱ�������״̬�������������趨ʱ�����������ӣ������ƽ̨����
************************************************************
*/
void NET_Timer(void)
{
	
	if(onenet_info.errCount >= NET_COUNT)								//������ʹ�������ﵽNET_COUNT��
	{
		UsartPrintf(USART_DEBUG, "Tips:	Timer Check Err-Send\r\n");
		
		onenet_info.errCount = 0;
		
		net_fault_info.net_fault_level = NET_FAULT_LEVEL_1;				//����ȼ�1
	}
	
	if(onenet_info.netWork == 0)											//����ڹ涨ʱ�������绹δ����ɹ�
	{
		if(++timerCount >= NET_TIME) 									//�������Ͽ���ʱ
		{
			UsartPrintf(USART_DEBUG, "Tips:	Timer Check Err-Init\r\n");
			
			timerCount = 0;
			
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_3;			//����ȼ�3
		}
	}
	else
	{
		timerCount = 0;													//�������
	}

}

/*
************************************************************
*	�������ƣ�	main
*
*	�������ܣ�	
*
*	��ڲ�����	��
*
*	���ز�����	0
*
*	˵����		
************************************************************
*/
int main(void)
{

	Hardware_Init();									//Ӳ����ʼ��
	
	NET_DEVICE_IO_Init();								//�����豸IO��ʼ��
	NET_DEVICE_Reset();									//�����豸��λ
	
	FW_Init();											//��ܲ��ʼ��
														//��������
	FW_CreateTask(USART_Task, 4);
	
	FW_CreateTask(STATUS_Task, 4000);
	
	FW_CreateTask(SEND_Task, 3000);
	
	FW_CreateTask(SENSOR_Task, 100);
	
	FW_CreateTask(DATA_Task, 10);
	
	FW_CreateTask(DATALIST_Task, 100);
	
	FW_CreateTask(FAULT_Task, 10);
	
	FW_CreateTask(NET_Task, 10);
	
	FW_CreateTask(CLOCK_Task, 20);
	
	FW_CreateTask(NET_Timer, 20);
	
	UsartPrintf(USART_DEBUG, "Running...\r\n");
	
	FW_StartSchedule();									//��ʼ�������

}
