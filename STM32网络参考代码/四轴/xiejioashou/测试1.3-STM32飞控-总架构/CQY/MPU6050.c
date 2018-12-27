/*************************************************
6�������Ǽ��ٶȴ�����MPU6050����

MCU��STM32F103CBT6��Ӳ��IIC���⺯���汾��3.5.0 KEIL

������2013-01-16

*************************************************/
#include "stm32f10x.h"
#include "IIC.h"
#include "var_global.h"


#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			  0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)


#define	MPU6050_Addr   0xD0	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸� 0x68

int16_XYZ ACC_RealData;
int16_XYZ GRY_RealData;

void MPU_GetData(u8 id,u8 reg,u8 len,u8 *buff);

void Init_MPU6050(void)
{

  I2C_WriteByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//�������״̬
	I2C_WriteByte(MPU6050_Addr,SMPLRT_DIV, 0x07); //������Ƶ7
	I2C_WriteByte(MPU6050_Addr,CONFIG, 0x04);     //25Hz��ͨ
	I2C_WriteByte(MPU6050_Addr,GYRO_CONFIG,  1<<3); //����������2000/s
	I2C_WriteByte(MPU6050_Addr,ACCEL_CONFIG, 1<<3); //���ٶ�����2G
	
}


void READ_MPU6050(void)
{
	u8 BUF_6050_buff[14];//6050���в�������
	
  MPU_GetData(MPU6050_Addr,ACCEL_XOUT_H,14,BUF_6050_buff);//������ȡ14���ֽ���ʱ500us
	
  
	ACC_RealData.X=(( ((int16_t)BUF_6050_buff[0]) <<8) |BUF_6050_buff[1]);
  ACC_RealData.Y=(( ((int16_t)BUF_6050_buff[2]) <<8) |BUF_6050_buff[3]);	
  ACC_RealData.Z=(( ((int16_t)BUF_6050_buff[4]) <<8) |BUF_6050_buff[5]);	
 
	GRY_RealData.X=(( ((int16_t)BUF_6050_buff[8]) <<8) |BUF_6050_buff[9]);
  GRY_RealData.Y=(( ((int16_t)BUF_6050_buff[10]) <<8) |BUF_6050_buff[11]);	
  GRY_RealData.Z=(( ((int16_t)BUF_6050_buff[12]) <<8) |BUF_6050_buff[13]);		
					  

}

void MPU_GetData(u8 id,u8 reg,u8 len,u8 *buff) 

{ //�������䷢���׼Ĵ�����ַ
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, id, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_Cmd(I2C1, ENABLE);
  I2C_SendData(I2C1,reg);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	//��ʼ����
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, id, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	

	while (len) 
	{   while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      
		  *buff =I2C_ReceiveData(I2C1);    
     
		 if(len==2){ 
		             I2C_AcknowledgeConfig(I2C1, DISABLE);//���һ����Ӧ��
		           }          		  
			buff++;
      len--;
  }
	
	
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

}










