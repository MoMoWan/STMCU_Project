/***********************************************

����: pid.c
����: ��������
��ַ��http://qiuyangdz.taobao.com
����: 2014/05/18
�汾��v1.0
����: pid����
˵����
*************************************************/
#include "stm32f10x.h"
#include "pid.h"
#include "motor.h"

pid_struct pid_yaw;
pid_struct pid_roll;
pid_struct pid_pitch;
/*************************************************

���ƣ�pid_init(pid_struct *pid)
���ܣ�pid��ʼ��
���������
    pid_struct *pid ��Ҫ��ʼ����pid  
�����������
����ֵ��  ��
**************************************************/
void pid_init(pid_struct *pid)
{
  pid->kp = 0.0f;
  pid->ki = 0.0f;
  pid->kd = 0.0f;

  pid->err0 = 0.0f;
  pid->err1 = 0.0f;
  pid->err2 = 0.0f;
  pid->sum_err = 0.0f;
  pid->zero_err = 0.0f;
  pid->output = 0.0f;
  pid->ki_max = 50.0f;
  pid->ki_min = -50.0f;
  pid->out_max = 800.0f;
  pid->out_min = -800.0f;
}
/*************************************************

���ƣ�pid_reset(void)
���ܣ�pid��λ����
�����������
�����������
����ֵ��  ��
**************************************************/
void pid_reset(void)
{  
  pid_pitch.err0 = 0;
  pid_pitch.err1 = 0;
  pid_pitch.err2 = 0;
  pid_pitch.sum_err = 0;
  pid_pitch.zero_err = 0;

  pid_roll.err0 = 0;
  pid_roll.err1 = 0;
  pid_roll.err2 = 0;
  pid_roll.sum_err = 0;
  pid_roll.zero_err = 0;

  pid_yaw.err0 = 0;
  pid_yaw.err1 = 0;
  pid_yaw.err2 = 0;
  pid_yaw.sum_err = 0;
  pid_yaw.zero_err = 0;  
}
/*************************************************

���ƣ�pid_config(void)
���ܣ�pid����
�����������
�����������
����ֵ��  ��
**************************************************/
void pid_config(void)
{
  pid_init(&pid_yaw);
  pid_init(&pid_roll);
  pid_init(&pid_pitch);
    
  pid_yaw.kp = -4.0f;
  pid_yaw.ki = -1.0f;
  pid_yaw.kd = -0.0f;
	
  pid_pitch.kp = -2.0f; 
  pid_pitch.ki = -0.04f;
  pid_pitch.kd = -2.0f;
	
  pid_roll.kp = 2.0f; 
  pid_roll.ki = 0.04f;
  pid_roll.kd = -2.0f;
}
/*************************************************

���ƣ�pid_calculate(pid_struct *pid, float angle, float angle_dot)
���ܣ�pid����
���������
    pid_struct *pid  ���������PID�ṹ��  
	float angle 	 ��ǰ�Ƕ�ֵ
	float angle_dot	 ��ǰ���ٶ�ֵ
�����������
����ֵ�� pid������ 
**************************************************/
float pid_calculate(pid_struct *pid, float angle, float angle_dot)
{
  float value_kp;	
  float value_ki;	
  float value_kd;	

  pid->err1 = pid->zero_err - angle;
  pid->sum_err = pid->sum_err + pid->ki * pid->err1;

  if(pid->sum_err > pid->ki_max)
  {
  	pid->sum_err = pid->ki_max;
  }
  if(pid->sum_err < pid->ki_min)
  {
  	pid->sum_err = pid->ki_min;
  }

  value_kp = pid->kp * pid->err1;
  value_ki = pid->sum_err;
  value_kd = pid->kd * angle_dot;

  pid->output = value_kp + value_ki + value_kd;

  if(pid->output > pid->out_max)
  {
    pid->output = pid->out_max;
  }
  else if(pid->output < pid->out_min)
  {
    pid->output = pid->out_min;
  }
  return (pid->output);
}
/*************************************************

���ƣ�calculate(float pitch, float roll, float yaw, u8 throttle, u8 rudder, u8 elevator, u8 aileron, float gx, float gy, float gz)
���ܣ�ϵͳpid����
���������
   	 float pitch   ��ǰ�����Ƕ�
	 float roll    ��ǰ��б�Ƕ�
	 float yaw     ��ǰƫ���Ƕ� 
	 u8 throttle   ���Ÿ���ֵ
	 u8 rudder     ƫ������ֵ
	 u8 elevator   ��������ֵ
	 u8 aileron    ��б����ֵ
	 float gx      x����ٶ�
	 float gy      y����ٶ�
	 float gz      z����ٶ�
������������������
����ֵ��  ��
**************************************************/
void calculate(float pitch, float roll, float yaw, u8 throttle, u8 rudder, u8 elevator, u8 aileron, float gx, float gy, float gz)
{

  u16 thr = 0;
  s16 pitch_out = 0, roll_out = 0, yaw_out = 0;
  s16 m1 = 0, m2 = 0, m3 = 0, m4 = 0;

  if(elevator > 0)  // �� 
  {
    if(elevator >= 0x80)
	{
	  elevator -= 0x80;
	  pid_pitch.zero_err = elevator * 0.2;
	}
    else  // ǰ 
	{
	  pid_pitch.zero_err = -(elevator * 0.2);
	}	 
  }
  else 
  {
    pid_pitch.zero_err = 0;
  }


  if(aileron > 0)  // ��   
  {
    if(aileron >= 0x80)
	{
	  aileron -= 0x80;
	  pid_roll.zero_err = -(aileron * 0.2);
	}
    else  // ��  
	{
	  pid_roll.zero_err = aileron * 0.2;
	}	 
  }
  else 
  {
    pid_roll.zero_err = 0;
  }


  if(rudder > 0)  // ��ƫ   
  {
    if(rudder >= 0x80)
	{
	  rudder -= 0x80;
	  pid_yaw.zero_err = rudder * 20.0;
	}
    else  // ��ƫ  
	{
	  pid_yaw.zero_err = -rudder * 20.0;
	}	 
  }
  else 
  {  
	pid_yaw.zero_err = 0;
  }

  roll_out = (s16)pid_calculate(&pid_roll, roll, gx);
  pitch_out = (s16)pid_calculate(&pid_pitch, pitch, gy);
  yaw_out = (s16)pid_calculate(&pid_yaw, yaw, gz);

  if(throttle > 0)  //  ����
  {
    thr = throttle * 7.0;
    m1 = thr - pitch_out + roll_out + yaw_out;
    m2 = thr + pitch_out + roll_out - yaw_out;
    m3 = thr + pitch_out - roll_out + yaw_out;
    m4 = thr - pitch_out - roll_out - yaw_out;
  }
  else  
  {
	m1 = 0;
	m2 = 0;
	m3 = 0;
	m4 = 0;
	pid_reset();
  }

  motor_control(m1, m2, m3, m4);  
}
/***************************END OF FILE**********************************************************************/

