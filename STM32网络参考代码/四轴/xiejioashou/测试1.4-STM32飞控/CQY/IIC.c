/*************************************************
STM32,Ӳ��IIC֧���ļ�

MCU��STM32F103CBT6���⺯���汾��3.5.0 KEIL

������2013-01-16

*************************************************/


#include "stm32f10x.h"
#include "IIC.h"

//----------------------------------------//


//-----------------------------------------------------------------//


void I2C_WriteByte(unsigned char id,unsigned char write_address,unsigned char byte)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	//������ʼ����
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	//�ȴ�ACK
	I2C_Send7bitAddress(I2C1,id,I2C_Direction_Transmitter);
	//���豸�����豸��ַ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	//�ȴ�ACK
	I2C_SendData(I2C1, write_address);
	//�Ĵ�����ַ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//�ȴ�ACK
	I2C_SendData(I2C1, byte);
	//��������
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//�������
	I2C_GenerateSTOP(I2C1, ENABLE);
	//���������ź�
}


//----------------------------------------------------//
unsigned char I2C_ReadByte(unsigned char  id, unsigned char read_address)
{  
	unsigned char temp; 	

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  	//�ȴ�I2C
  	I2C_GenerateSTART(I2C1, ENABLE);
  	//������ʼ�ź�
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    //EV5
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Transmitter);
	//���͵�ַ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  	//EV6
  	I2C_Cmd(I2C1, ENABLE);
 	//�������ÿ������EV6
  	I2C_SendData(I2C1, read_address);  
	//���Ͷ��õ�ַ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	//EV8 
  	I2C_GenerateSTART(I2C1, ENABLE);
	//���·���
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  	//EV5
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Receiver);
  	//���Ͷ���ַ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  	//EV6  
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
	//�ر�Ӧ���ֹͣ��������
    while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
	      
    temp = I2C_ReceiveData(I2C1);
   
  	I2C_AcknowledgeConfig(I2C1, ENABLE);
		
    return temp;
}
//-----------------------------------------//
