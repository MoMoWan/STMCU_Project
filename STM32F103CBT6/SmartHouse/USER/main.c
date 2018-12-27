#include "main.h"


int main(void)
{	
	uint16 times=0;
	
	mDelay(1000);
	LED_Init();
	House_Init();
	Delay_Init();
	ESP8266_IO_Init();
	IIC_Init();
	USART1_Init();
	USART2_Init();
	RC522_Init ();     //RC522ģ����������ĳ�ʼ������
	PcdReset ();
	M500PcdConfigISOType ( 'A' );//���ù�����ʽ
	
	ESP8266_Reset();		
	ESP8266_Init();
	LED_Switch(LED_ON,LED_R|LED_G|LED_Y|LED_B);
	
	

	BEE_ON;	
	DelayMs(250);
	BEE_OFF;
	
	while(1)
	{
		times++;
		Status_Scan();
		if(times%500==0)
		{
//			ESP8266_CheckStatus(20);
//			Ping_Server();	
			OneNet_DevLink(DEVICEID,APIKEY);    
		}else if(times%20==0)
		{
			SHT20_GetValue();						
			Save_DataToOneNet();	
		} 
		
		IC_test();//IC�����
	}  
	
}
