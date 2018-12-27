/* Includes ------------------------------------------------------------------*/
#include "esp8266.h"

#include "EDP_Protocol.h"  

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"

WIFI_Frame ESP_Frame;
uint8_t UART_TX_Buf[128];

uint8_t Dat[35]={
                  0X80,
	                0X21,//��Ϣ����
	                0X80,
	                0X00,0X08,
	                0X34,0X35,0X30,0X31,0X33,0X39,0X35,0X33,
	                0X05,
	                0X00,0X13,//�·����ݳ���
	                0X2C,0X3B,
                  0X44,0X30,0X31,0X2C,//D01,
	                0,0,0X2E,0,//00.0
	                0X3B,//;
	                0X44,0X30,0X32,0X2C,//D02,
                  0,0,0X2E,0,//00.0      
};

/*
0Ϊ�ɹ�����AT����
1Ϊʧ��
*/
uint8_t WIFI_ATCMD(char* CMD,char * Token1,char * Token2,uint16_t timems)
{
	 uint8_t Token1_Flag,Token2_Flag,Err_Flag;
	 uint16_t Len;
	
	 Token1_Flag=0;Token2_Flag=0;
	 Err_Flag=0;
	 Len=strlen(CMD);
	 memcpy(UART_TX_Buf,(uint8_t *)CMD,Len);
	
  if( HAL_UART_Transmit(&huart2,UART_TX_Buf, Len, 0xFF) !=HAL_OK )
		{
			return 1;
		}

		  HAL_Delay(timems);
			if( ( Token1 == 0 ) && ( Token2 == 0 ) )
			{
			return 0;
			}
			else
			{
					if (strstr((char *)WIFI_RX_Buffer, Token1) != NULL)
					{
					 Token1_Flag=0;
					}
					else
					{
					Token1_Flag=1;
					}
					if(strstr((char *)WIFI_RX_Buffer, Token2) != NULL)
					{
					Token2_Flag=0;
					}
					else
					{
					Token2_Flag=1;
					}		
					
					if (strstr((char *)WIFI_RX_Buffer, "ERROR\r\n") != NULL)
					{
					 Err_Flag=1;
					}
					else
					{
					 Err_Flag=0;
					}
					
					if( (Token1_Flag==0 || Token2_Flag==0) && Err_Flag==0 )
					{
//					 memset(WIFI_RX_Buffer,0,1024);
					 return 0;
					}
					else
					{
					 return 1;
					}
					
			}
			
//	return 1; 
}




/*
0Ϊ�ɹ�����WIFI�����
1Ϊʧ��
*/
uint8_t WIFI_JAP_Type1(char* ssid, char* Password)//AT+CWJAP="ssid","Password"
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, Password);
  if( WIFI_ATCMD(tmp,"OK",NULL,10000) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}


/*
0Ϊ�ɹ�����WIFI�����
1Ϊʧ��
*/
uint8_t WIFI_JAP_Type2(char* ssid, char* Password)//AT+CWJAP="\"ssid\"","Password"
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWJAP=\"\\\"%s\\\"\",\"%s\"\r\n", ssid, Password);
  if( WIFI_ATCMD(tmp,"OK",NULL,10000) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}




/*
0Ϊ�ɹ�����ESP����ģʽ
1Ϊʧ��
1:Stationģʽ 
2:AP ģʽ 
3:AP��Stationģʽ
AT+CWMODE=Mode
*/
uint8_t WIFI_CWMODE(uint8_t Mode)
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWMODE=%d\r\n",Mode);
  if( WIFI_ATCMD(tmp,"OK",NULL,2) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}






/*
����ONENET IP
0Ϊ�ɹ�����ONENET IP
1Ϊʧ��
*/
uint8_t WIFI_ConnectIP(void)
{
	 char tmp[128];
	 sprintf( tmp, "AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n");
  if( WIFI_ATCMD(tmp,"CONNECT","ALREADY CONNECTED",1000) !=0 )
		{
		return 1;
		}			
		else
		{
//			if( 	WIFI_ATCMD("AT+CIPSTATUS","STATUS:3",NULL,1) !=0 )
//			{
//			 return 1;
//			}
//			else
//			{
			return 0;
//			}

		}
}





 




/*
����ONENET IP
0Ϊ�ɹ�����ONENET IP
1Ϊʧ��
*/
uint8_t WIFI_ConnectOnenet(uint8_t* buf)
{
   char cmd[32];
	 
	 sprintf(cmd,"AT+CIPSEND=%d\r\n",buf[1]+2);
	
	 if( WIFI_ATCMD(cmd,">",NULL,10) !=0 )
		{
		return 1;
		}			
		else
		{
	    HAL_UART_Transmit(&huart2,buf,buf[1]+2,0xff);
			if( WIFI_ATCMD(NULL,"SEND OK",NULL,5)!=0 )
			{
			return 1;
			}
			else
			{
			return 0;
			}
		}
}


/*
����ONENET IP
0Ϊ�ɹ�����ONENET IP
1Ϊʧ��
*/
uint8_t WIFI_Onenet_Send(float D01,float D02)
{
	  char tmp_Buf[4];
	
		sprintf( tmp_Buf,"%2.1f",D01);
	  Dat[22]=tmp_Buf[0];
	  Dat[23]=tmp_Buf[1];
	  Dat[25]=tmp_Buf[3];
	
		sprintf( tmp_Buf,"%2.1f",D02);
	  Dat[31]=tmp_Buf[0];
	  Dat[32]=tmp_Buf[1];
	  Dat[34]=tmp_Buf[3];
	
   
		if( WIFI_ATCMD("AT+CIPSEND=35\r\n",">",NULL,5) !=0 )
		{
		return 1;
		}			
		else
		{
	  HAL_UART_Transmit(&huart2,(uint8_t *)Dat,35,0xff);
			if( WIFI_ATCMD( NULL,"SEND OK",NULL,5)!=0 )
			{
			return 1;
			}
			else
			{
				return 0;	
			}
		}
}


uint8_t WIFI_Onenet_RecD03(void)
{
  char *p;
	 uint8_t D03;
	
	 p=(char *)&WIFI_RX_Buffer;
	 p = strstr(p,"D03");
	 if( p )
		{
		  p+=3;
			 D03=atoi(p);		
		}
		return D03;
}

uint8_t WIFI_Onenet_RecD04(void)
{
  char *p;
	 uint8_t D04;
	
		p=(char *)&WIFI_RX_Buffer;
		p = strstr(p,"D04");
	 if( p )
		{
		  p+=3;
			 D04=atoi(p);		
		}
		return D04;
}

uint8_t WIFI_Onenet_RecD05(void)
{
  char *p;
	 uint8_t D05;
	
		p=(char *)&WIFI_RX_Buffer;
		p = strstr(p,"D05");
		if( p )
		{
		  p+=3;
			 D05=atoi(p);		
		}
		
		return D05;
}

