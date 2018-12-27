#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"
#include "mpu6050.h"
#include "pwm_out.h" 
#include <math.h>
#include "usart.h"
extern u8        lock;   //�ɿ�������־λ
extern int16_t moto1,moto2,moto3,moto4;

typedef struct
{
	float pitch;//������̬
	float roll;
	float yaw;
	float THROTTLE;//����
}T_RC_Dat;//ң��������

extern T_RC_Dat  Rc_D;//ң��ͨ������;
//�⻷PID����
extern float Pitch_shell_kp;
extern float Pitch_shell_kd;
extern float Pitch_shell_ki;
/*******************************/
extern float Roll_shell_kp;
extern float Roll_shell_kd;            
extern float Roll_shell_ki;
/*******************************/
extern float Yaw_shell_kp;
extern float Yaw_shell_kd;      
extern float Yaw_shell_ki;
//�ڻ�PD����

extern float Pitch_core_kp;
extern float Pitch_core_ki;
extern float Pitch_core_kd;

extern float Roll_core_kp;
extern float Roll_core_ki;
extern float Roll_core_kd;

extern float Yaw_core_kp;
extern float Yaw_core_ki;
extern float Yaw_core_kd;
extern float RC_Pitch,RC_Roll,RC_Yaw;           //��������̬��
extern S_INT16_XYZ MPU6050_GYRO_LAST;
void CONTROL_PID(float pit, float rol, float yaw);
void ahrs_control_PID_moto(void);//�ؼ��֣����� ���� ����PID ���
#endif




