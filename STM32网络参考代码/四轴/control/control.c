/**********************************************************************************************************************
******����PID���Ƴ����⻷���ýǶȿ��ƣ��ڻ����ý��ٶȿ��ƣ�ֱ��������������� �⻷����Ϊŷ���ǣ�Ӳ��DMP��*****
***********************************************************************************************************************
******���ӳ���Ϊ��������������㷨 ********************************************************************************/
/****************************
					 Y(Roll)
     ˳ʱ��ת | ��ʱ��ת
      motor1  |  motor4
       ����   |   ����
    --------------------X(Pitch)          
     ��ʱ��ת | ˳ʱ��ת 
      motor2  |  motor3
       ����   |   ���� 
              |
****************************/
#include "control.h" 
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "key.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "ahrs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
 
//����Ҫ���µ�ACC���ٶ� GYRO������
S_INT16_XYZ MPU6050_GYRO_LAST;

T_RC_Dat  Rc_D={0,0,0,1613};//ң��ͨ������;
u8 lock=0;   //�ɿ�������־λ
extern u8 txbuf[4];  //���ͻ���
extern u8 rxbuf[4];  //���ջ���
extern u16 test1[3]; //���յ�NRf24L01����

float thr=0;//����


//PID1 PID_Motor;
/***************************************************************/
float Pitch_i,Roll_i,Yaw_i;              //������
float Pitch_old,Roll_old,Yaw_old;        //�Ƕȱ���
float Pitch_d,Roll_d,Yaw_d;              //΢����

float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;           //��������̬��

float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//�⻷�����
        //�⻷PID����
float Pitch_shell_kp=140;
float Pitch_shell_ki=0.2;
float Pitch_shell_kd=0.8;
/*********************************/
float Roll_shell_kp=150;              
float Roll_shell_ki=0.2;
float Roll_shell_kd=0.8; 
/*********************************/	
float Yaw_shell_kp=15;//0.87              
float Yaw_shell_ki=0.002;
float Yaw_shell_kd=0.02;   
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//�����Ǳ���
float pitch_core_kp_out,pitch_core_ki_out,pitch_core_kd_out,Roll_core_kp_out,\
			Roll_core_ki_out,Roll_core_kd_out,Roll_core_ki_out,Yaw_core_kp_out,\
			Yaw_core_ki_out,Yaw_core_kd_out;//�ڻ��������

float Pitch_core_out,Roll_core_out,Yaw_core_out;//�ڻ������       
       
//�ڻ�PID����
float Pitch_core_kp=0.02;
float Pitch_core_ki=0;
float Pitch_core_kd=0.001;//0.001

float Roll_core_kp=0.02;
float Roll_core_ki=0;
float Roll_core_kd=0.001;

float Yaw_core_kp=0.001;
float Yaw_core_ki=0;
float Yaw_core_kd=0.001;


int16_t moto1=1613,moto2=1613,moto3=1613,moto4=1613;//�����PWMֵ


/////////////////////////////////////////////////////////////////
//PID ���ƺ���
//���룺��������̬ŷ��������
//���أ���
//��ע��Pitch��Roll_i�����ڻ����⻷PID���� ��yaw�����ڻ�����
////////////////////////////////////////////////////////////////
void CONTROL_PID(float pit, float rol, float yaw)
{

  RC_Pitch=(Rc_D.pitch -1499.0f )/50;
  RC_Pitch-=3.3f;
//////////////////�⻷(PID)/////////////
  Pitch_i+=(pit-RC_Pitch);
//-------------Pitch�����޷�----------------//
  if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
//-------------Pitch΢��--------------------//
  Pitch_d=pit-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(pit-RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//����Ƕ�
  Pitch_old=pit;
/*********************************************************/       
       
	RC_Roll=(Rc_D.roll-1502)/50;
	Roll_i+=(rol-RC_Roll);
//-------------Roll�����޷�----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll΢��--------------------//
  Roll_d=rol-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(rol-RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll����Ƕ�------------------//
  Roll_old=rol;
       
       
  //RC_Yaw=(Rc_D.yaw-1501)*10;
//-------------Yaw΢��--------------------//
  Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Roll  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(MPU6050_GYRO_LAST.Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll����Ƕ�------------------//
  Yaw_old=MPU6050_GYRO_LAST.Z;

////////////////////////�ڻ����ٶȻ�(PD)/////////////////////////////// 
//������	����ǽ��ٶ�
  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + MPU6050_GYRO_LAST.Y );
  pitch_core_kd_out = Pitch_core_kd * (MPU6050_GYRO_LAST.Y   - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + MPU6050_GYRO_LAST.X );
  Roll_core_kd_out  = Roll_core_kd  * (MPU6050_GYRO_LAST.X   - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + MPU6050_GYRO_LAST.Z);
  Yaw_core_kd_out  = Yaw_core_kd  * (MPU6050_GYRO_LAST.Z   - Gyro_radian_old_z);
       
       
  Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
  Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

  Gyro_radian_old_y = MPU6050_GYRO_LAST.X;
  Gyro_radian_old_x = MPU6050_GYRO_LAST.Y;
  Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   

       
//--------------------�����ֵ�ںϵ��ĸ����--------------------------------//
				       
        if(Rc_D.THROTTLE>1613)
        {
					thr=Rc_D.THROTTLE;               
					moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
					moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
					moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
					moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);  
						
				}
        else
        {
					moto1 = 1613;
					moto2 = 1613;
					moto3 = 1613;
					moto4 = 1613;
        }
        Motor_PWM_Update(moto1,moto2,moto3,moto4);
}


void ahrs_control_PID_moto(void)//�ؼ��֣����� ���� ����PID ���
{
	if(lock==1)
		{

		  if(mpu_dmp_get_data(&Angle.pitch ,&Angle.roll,&Angle.yaw)==0)//�õ�ŷ���� 
			{
				//MPU_Get_Accelerometer(&Acc.X ,&Acc.Y ,&Acc.Z );	//�õ����ٶȴ���������
        MPU_Get_Gyroscope(&Gyro.X ,&Gyro.Y ,&Gyro.Z);//��ȡ����������
				MPU_Get_GYRO();//���������ݸ��º���
				CONTROL_PID(Angle_SET.pitch,Angle_SET.roll,Angle_SET.yaw);//PID����pwmֵ	
			  if(report) Data_Exchange();//�������ݸ��µ�������λ��
			}
				
		}
		else//�ɻ�����
		{
			     Motor_PWM_Update(moto1-100,moto2-100,moto3-100,moto4-100);//�ɿ���������û����
		}
}














