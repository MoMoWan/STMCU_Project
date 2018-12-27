#include "stm32f10x.h"
#include "delay.h"
#include "math.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "ALL_config.h"
#include  "PID.h"
#include "var_global.h"

float PID_Dt; 

PID Pitch,Roll,Yaw;
PID PitchRate,RollRate,YawRate;

float PitchOutput,RollOutput,YawOutput;

float rollRateDesired;
float pitchRateDesired;
float yawRateDesired;

void pidInit(PID* pid, const float desired, const float kp,
             const float ki, const float kd)
{
  pid->Err = 0;
  pid->PreErr = 0;
  pid->integ = 0;
  pid->deriv = 0;
  
	pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
  pid->I_Max = 100;
}

void PID_init_Eur(float p,float i,float d)
{
  Pitch.kp=p;
  Roll. kp=p;
	
	
	Pitch.ki=i;
  Roll. ki=i;

	
	Pitch.kd=d;
  Roll. kd=d;

  PID_Dt=0.005f;
	Pitch.I_Max=20;
	Roll.I_Max=20;
}

void PID_init_Rate(float p,float i,float d)
{
  PitchRate.kp=p;
  RollRate. kp=p;
	
	
	PitchRate.ki=i;
  RollRate. ki=i;
	
	PitchRate.kd=d;
  RollRate. kd=d;
	
	PitchRate.I_Max=500;
	RollRate.I_Max =500;
 
}

void PID_Yaw_Rate(float p,float i,float d)
{
	YawRate.  kp=p;

  YawRate.  ki=i;
	
	YawRate.  kd=d;
	
  YawRate. I_Max =500;
	
	
}

float  pidUpdate(PID* pid, const float measured, const u8 updateError)//PID����
{
  float output;
 
 if (updateError)//�Ƿ������true��ʱ�����err
  {
     pid->Err = pid->desired - measured; 
  }
	
  pid->integ += pid->Err * PID_Dt; //���ֲ��� �ۼӣ����*�������ڣ�
  if (pid->integ > pid->I_Max)
  {
    pid->integ = pid->I_Max;
  }
  else if (pid->integ < -pid->I_Max)
  {
    pid->integ = -pid->I_Max;
  }
  //���ֱ�������
                                
  pid->deriv = (pid->Err - pid->PreErr) / PID_Dt;//΢�ּ��㣬�������ȥ�ϴ����/��������

  pid->outP = pid->kp * pid->Err;//�������㣬P*���
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  output = (pid->kp * pid->Err) +
           (pid->ki * pid->integ) +
           (pid->kd * pid->deriv);  //PID�����PID����ĺ�

  pid->PreErr = pid->Err;//����ǰ�����

  return output;
}

//--------------------------------------------------------------------------//
void pidSetDesired(PID* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PID* pid)
{
  return pid->desired;
}

void pidSetError(PID* pid, const float error)
{
  pid->Err = error;
}

 void controllerCorrectRatePID(  //���ٶ�PID���ƣ�����PID��
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&RollRate, rollRateDesired);//����PID����ֵ
  
	RollOutput=pidUpdate(&RollRate, rollRateActual,1);//����PID���������������

  pidSetDesired(&PitchRate, pitchRateDesired);
  PitchOutput=pidUpdate(&PitchRate, pitchRateActual,1);
	
	 pidSetDesired(&YawRate, yawRateDesired);
   YawOutput=pidUpdate(&YawRate, yawRateActual, 1);


}

void controllerCorrectAttitudePID(//��̬��PID
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{ 
	float yawError;//�������Ҫ��������·�� 

  pidSetDesired(&Roll, eulerRollDesired);//����PID����ֵ����PID��
  *rollRateDesired = pidUpdate(&Roll, eulerRollActual,1);//����PID�����ѽ��������ٶ���������������ǿ����Ӧ�Ƕȱ仯


  pidSetDesired(&Pitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&Pitch, eulerPitchActual,1);
	
  
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&Yaw, yawError);
  *yawRateDesired = pidUpdate(&Yaw, eulerYawActual, 0);



}

void PID_RPY_Rate()
{
	
	yawRateDesired = EXP_ANGLE.Z;
	
 controllerCorrectRatePID(-GRY_F.X, GRY_F.Y, GRY_F.Z,
                          rollRateDesired, pitchRateDesired, yawRateDesired);//������ٶȻ�PID��ʹ�ý��ٶȼ��㣬ִ�д�������

}

void PID_Eur()
{
	
 controllerCorrectAttitudePID(Q_ANGLE.Rool, Q_ANGLE.Pitch, Q_ANGLE.Yaw,
                              EXP_ANGLE.X, EXP_ANGLE.Y, 0,
                              &rollRateDesired, &pitchRateDesired, &yawRateDesired);   //������̬��PID
                              //�������������
}
