#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "stm32f10x.h"
#include "SysTick.h"


//IO����
#define SCL_H         GPIOC->BSRR = GPIO_Pin_12
#define SCL_L         GPIOC->BRR  = GPIO_Pin_12 
    
#define SDA_H         GPIOC->BSRR = GPIO_Pin_11
#define SDA_L         GPIOC->BRR  = GPIO_Pin_11

#define SCL_read      GPIOC->IDR  & GPIO_Pin_12 
#define SDA_read      GPIOC->IDR  & GPIO_Pin_11


//IIC���в�������
void I2C_Init_IO(void);               //��ʼ��IIC��IO��				 
void I2C_Start(void);				//����IIC��ʼ�ź�
void I2C_Stop(void);	  			//����IICֹͣ�ź�

bool I2C_WaitAck(void); 				//IIC�ȴ�ACK�ź�
void I2C_Ack(void);					//IIC����ACK�ź�
void I2C_NoAck(void);				//IIC������ACK�ź�

void I2C_SendByte(u8 SendByte);			//IIC����һ���ֽ�
u8 I2C_ReadByte(void);            //IIC��ȡһ���ֽ�

//******���ֽ�д��**********//
bool Single_Write(u8 REG_Address,u8 REG_data,u8 SlaveAddress);
//********���ֽڶ�ȡ*********//
u8 Single_Read(u8 REG_Address,u8 SlaveAddress);
//���ֽ�д��//
void Multiple_write(u8 star_addr,u8 num,u8 SlaveAddress,u8* send_buf);
//��ȡ����ֽڵ�����//
void Multiple_read(u8 star_addr,u8 num,u8 SlaveAddress,u8* recv_buf);

/////////////////////////////mpu6050�ӿں���///////////////////////////////////////////////////////////
void i2cRead(u8 SlaveAddress,u8 star_addr,u8 num,u8* recv_buf);
void i2cWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data);

#endif


