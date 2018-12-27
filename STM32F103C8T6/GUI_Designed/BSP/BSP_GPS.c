#include "BSP_GPS.h"

#include "string.h"

#include "main.h"
#include "stm32f1xx_hal.h"


extern UART_HandleTypeDef huart2;
extern uint8_t UART_GPS_RX[512];
uint8_t UTC_TO_BJT[24]={
8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24
};
	
	


GPS_GPRMC_INFO Get_GPS_GPRMC(void)
{
	
 
  char * p;
		
 p=strstr( (const char*)UART_GPS_RX,"RMC");  
      if(p)
			{
						p = strchr(p, ',');
						
						p++;
						GPRMC.GPS_UTC_Hours =   StrToIntFix(p, 2);
				    GPRMC.GPS_UTC_Hours=UTC_TO_BJT[GPRMC.GPS_UTC_Hours];
						p += 2;
						GPRMC.GPS_UTC_Minutes = StrToIntFix(p, 2);
						p += 2;
						GPRMC.GPS_UTC_Seconds = StrToIntFix(p, 2);
						p += 3;

							/* �ֶ�2 ״̬��A=��λ��V=δ��λ */
						p = strchr(p, ',');
						p++;
						
						
						
						if (*p != 'A')
						{
						  GPRMC.GPS_Statu = 1;
						}
						
            else
						{
						
						
						
						  GPRMC.GPS_Statu = 2;
							
											/* �ֶ�3 γ��ddmm.mmmmm���ȷָ�ʽ��ǰ��λ��������0�� */
									p = strchr(p, ',');

									p++;
									
							    GPRMC.GPS_Lagitude_DD=StrToIntFix(p, 2);
							    p += 2;
							
									GPRMC.GPS_Lagitude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Lagitude_MM += StrToIntFix(p, 5);
									p += 5;
							

									/* �ֶ�4 γ��N����γ����S����γ��*/
									p = strchr(p, ',');
									if (p == 0)
									{
										GPRMC.GPS_NS = 2;
									}
									p++;
									
									if (*p == 'S')
									{
										GPRMC.GPS_NS = 2;
									}
									
									else if (*p == 'N')
									{
										GPRMC.GPS_NS = 1;
									}


									/* �ֶ�5 ����dddmm.mmmmm���ȷָ�ʽ��ǰ��λ��������0�� */
									p = strchr(p, ',');
									

									p++;

							    GPRMC.GPS_Longititude_DD=StrToIntFix(p, 3);
							    p += 3;
							
									GPRMC.GPS_Longititude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Longititude_MM += StrToIntFix(p, 5);
									p += 5;

									/* �ֶ�6������E����������W�������� */
									p = strchr(p, ',');
									if (p == 0)
									{
										GPRMC.GPS_WE=2;
									}
									p++;
									
									if (*p == 'E')
									{
										GPRMC.GPS_WE = 2;
									}
									
									else if (*p == 'W')
									{
										GPRMC.GPS_WE = 1;
									}
							
						}

						
						
  
			
			}
	
	return GPRMC;
 
}




int32_t StrToInt(char *_pStr)
{
	uint8_t flag;
	char *p;
	uint32_t ulInt;
	uint8_t i;
	uint8_t ucTemp;

	p = _pStr;
	if (*p == '-')
	{
		flag = 1;	/* ���� */
		p++;
	}
	else
	{
		flag = 0;
	}

	ulInt = 0;
	for (i = 0; i < 15; i++)
	{
		ucTemp = *p;
		if (ucTemp == '.')	/* ����С���㣬�Զ�����1���ֽ� */
		{
			p++;
			ucTemp = *p;
		}
		if ((ucTemp >= '0') && (ucTemp <= '9'))
		{
			ulInt = ulInt * 10 + (ucTemp - '0');
			p++;
		}
		else
		{
			break;
		}
	}

	if (flag == 1)
	{
		return -ulInt;
	}
	return ulInt;
}










int32_t StrToIntFix(char *_pStr, uint8_t _ucLen)
{
	uint8_t flag;
	char *p;
	uint32_t ulInt;
	uint8_t i;
	uint8_t ucTemp;

	p = _pStr;
	if (*p == '-')
	{
		flag = 1;	/* ���� */
		p++;
		_ucLen--;
	}
	else
	{
		flag = 0;
	}

	ulInt = 0;
	for (i = 0; i < _ucLen; i++)
	{
		ucTemp = *p;
		if (ucTemp == '.')	/* ����С���㣬�Զ�����1���ֽ� */
		{
			p++;
			ucTemp = *p;
		}
		if ((ucTemp >= '0') && (ucTemp <= '9'))
		{
			ulInt = ulInt * 10 + (ucTemp - '0');
			p++;
		}
		else
		{
			break;
		}
	}

	if (flag == 1)
	{
		return -ulInt;
	}
	return ulInt;
}

