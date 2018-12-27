#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//Mini STM32������
//SPI ���� V1.1
//����ԭ��@ALIENTEK
//2010/5/13	

// SPI�����ٶ����� 
#define SPI_SPEED_2   0
#define SPI_SPEED_4   1
#define SPI_SPEED_8   2
#define SPI_SPEED_16  3
#define SPI_SPEED_256 4
						  	    													  
void SPIx_Init(SPI_TypeDef *SPIx);			 //��ʼ��SPI��
void SPIx_SetSpeed(SPI_TypeDef *SPIx,u8 SpeedSet); //����SPI�ٶ�   
u8 SPIx_ReadWriteByte(SPI_TypeDef *SPIx,u8 TxData);//SPI���߶�дһ���ֽ�
		 
#endif

