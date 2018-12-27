/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��mpu6050.c
 * ����    ��mpu6050����         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

u8		 mpu6050_buffer[14];//����λ���顿=================��IIC��ȡ�������ݡ�	
struct _sensor sensor;      //�˴�����һ�������ṹ

/*struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };
              
  struct _trans{
    struct _int16 origin;  //ԭʼֵ
    struct _float averag;  //ƽ��ֵ
    struct _float histor;  //��ʷֵ
    struct _int16 quiet;   //��ֵ̬
    struct _float radian;  //����ֵ 
          };                        
*/

/*====================================================================================================*/
/*====================================================================================================*
**���� : InitMPU6050
**���� :����ʼ��MPU6050��
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
u8 InitMPU6050(void)
{
	u8 ack;
	//MPU6050���ӻ��豸�а�λ��ַ  ����λ0X68�洢��Who am I�� ��һλ��AD0����
	ack = Single_Read(MPU6050_ADDRESS, WHO_AM_I);                      //���MPU6050�Ƿ����Ӻ�
	if (!ack)    //����ack=0X68��    !ack=0��   ִ�г�ʼ��
    return FALSE;
	
	Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	               //�������״̬
    //SMPLRT_DIV	0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)          //8ms�ɼ�һ��
	Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	Single_Write(MPU6050_ADDRESS, CONFIGL, MPU6050_DLPF);              //��ͨ�˲�
	Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, MPU6050_GYRO_FS_1000);  //���������� +-1000
	Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, MPU6050_ACCEL_FS_4);   //���ٶ����� +-4G
	return TRUE;
}

//**************************ʵ�ֺ���********************************************
//  ����iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���,����MPU6050_Last��
//******************************************************************************
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, 0x3B);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, 0x3C);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, 0x3D);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, 0x3E);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, 0x3F);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, 0x40);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, 0x43);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, 0x44);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, 0x45);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, 0x46);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, 0x47);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, 0x48);
	
}
/**************************ʵ�ֺ���********************************************
//        ��MPU6050�������� ��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ�����
//        �����û�н��м��ٶ���ƫ��������м��㡿
*******************************************************************************/
void MPU6050_Dataanl(void)       //MPU6050�������Ϊ16λ
{
	MPU6050_Read();
	                             //���������ݽ��кϲ�����ƫ
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x; // �����ٶȼ����ݡ�
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);                      //Z����ٶȼ�δ����ƫ

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);                     // �����������ݡ�
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
    //������ƫ�ֿ����ĺô� ��Ϊ �±�������ƫ���㺯��Ҫ�õ����ݳ�ʼֵ
	sensor.gyro.radian.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;                                    // �����������ݼ���ƫ��
	sensor.gyro.radian.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.radian.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;

//    	The calibration  of  acc                      //�����ٶȼ���ƫ���㡿
//      �˴����ٶȼ������ʵ���ۼӼ����
//      �˺�������̬���㺯���е��� ÿ�ж�һ��ִ��һ��
//      ͨ���ж���ѯ�Ӿ�̬�ֲ�����ʵ���ۼ�
//*****************************************************************************//	
	 if(flag.calibratingA)                            //����־λ�� ������ٶ���ƫ����û�����  flag.calibratingA��rc.c�ж���
	 {
		 static int32_t	tempax=0,tempay=0,tempaz=0;   //�������ٶ���ƫ
		 static uint8_t cnt_a=0;
		 if(cnt_a==0)                                 //�״�����
		 {
				sensor.acc.quiet.x = 0;
				sensor.acc.quiet.y = 0;
				sensor.acc.quiet.z = 0;
				tempax = 0;
				tempay = 0;
				tempaz = 0;
				cnt_a = 1;
		 }
				tempax+= sensor.acc.origin.x;
				tempay+= sensor.acc.origin.y;
				tempaz+= sensor.acc.origin.z;
				if(cnt_a==200)                          //�ۼ�200����ƽ��ֵ
				{
					sensor.acc.quiet.x = tempax/cnt_a;
					sensor.acc.quiet.y = tempay/cnt_a;
					sensor.acc.quiet.z = tempaz/cnt_a;
					cnt_a = 0;
					flag.calibratingA = 0;
					EE_SAVE_ACC_OFFSET();               //�Ѽ���ļ��ٶ���ƫֵ������EEPROM��
					return;
				}
				cnt_a++;		
			}	
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Gyro_Calculateoffest
**���� : ��������������ƫ��
**���� : 
**��� : None
**ʹ�� : Hto_Gyro_Calculateoffest();
**====================================================================================================*/
/*====================================================================================================*/
void Gyro_Caloffest(s32 x,s32 y,s32 z,u16 amount)
{
     sensor.gyro.quiet.x = x/amount;
	 sensor.gyro.quiet.y = y/amount;
	 sensor.gyro.quiet.z = z/amount;
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : Gyro_OFFSET    
**���� :�������Ǿ�̬�ɼ��� ���� ��������������ƫ��
**���� : None      ����״̬�²ɼ������������ƫֵ
**ݔ�� : None      50����ƽ��  ����ִ�� ѭ��ʵ��
**��ע : None     ��������ʼ��ʱִ�С�
**====================================================================================================*/
/*====================================================================================================*/
//��ѭ����ȷ�����ᴦ����ȫ��ֹ״̬    ��־λֻ�ں�����ʹ��
void Gyro_OFFSET(void)
{
	static u8 over_flag=0;                           //��u8  over_flag ��
	u8  i,cnt_g = 0;                                 //��u8  i  , cnt_g��
	s32 Integral[3] = {0,0,0};                       //��s32 Integral[3]��
	s32 tempg[3]={0,0,0};                            //��s32   tempg[3] ��
	s16 gx_last=0,gy_last=0,gz_last=0;               //��s16 gx_last=0  , gy_last=0 , gz_last=0��

	while(!over_flag)	                   
	{
		if(cnt_g < 50)
		{
			MPU6050_Dataanl();                //������ƫ�ĳ�ʼֵ����   

			tempg[0] += sensor.gyro.origin.x; //�˴�ʹ�õ������ݳ�ʼֵ ����û��Ҫ������ƫ��ֵ������
			tempg[1] += sensor.gyro.origin.y;
			tempg[2] += sensor.gyro.origin.z;
            
            //absu16( Math_X )  (Math_X<0? -Math_X:Math_X)
			Integral[0] += absu16(gx_last - sensor.gyro.origin.x);  // ǰ�β���ֵ �� ���β���ֵ  ÿ�ζ�ȡ��
			Integral[1] += absu16(gy_last - sensor.gyro.origin.y);    
			Integral[2] += absu16(gz_last - sensor.gyro.origin.z);

			gx_last = sensor.gyro.origin.x;
			gy_last = sensor.gyro.origin.y;                  
			gz_last = sensor.gyro.origin.z;
		}
		else{
			// δУ׼�ɹ�  GYRO_GATHER   100 ������β���ֵ��ֵ������̬����ʧ��   �˴�������̬�������������� ����ƫ����ܹ���
			if(Integral[0]>=GYRO_GATHER || Integral[1]>=GYRO_GATHER || Integral[2]>= GYRO_GATHER){
				cnt_g = 0;
				for(i=0;i<3;i++){
				tempg[i]=Integral[i]=0;  //��� �ۼ�����
				}
			}
			// У׼�ɹ� 
			else{				
				   Gyro_Caloffest(tempg[0],tempg[1],tempg[2],50); //������������ƫֵ
				   over_flag = 1;
			}
		}
		cnt_g++;
	}
}



