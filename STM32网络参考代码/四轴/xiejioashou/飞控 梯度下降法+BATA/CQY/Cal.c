#include "stm32f10x.h"
#include "math.h"
#include "var_global.h"
#include "cal.h"
#include "stdlib.h"

//#define Gyr_Gain 	0.015267       //���ٶ�500���� 
#define Gyr_Gain 	0.0305f      //���ٶ�1000���� 
//#define Gyr_Gain 	0.0610f      //���ٶ�2000���� 


#define ACC_Gain 	0.0011963f 
#define AVGtimes  10
float invSqrt(float x);
u16 Data_time;

u8 AVG_cnt;

float_XYZ ACC_F,GRY_F;

float_XYZ ACC_AVG;

int16_XYZ ACC_Offset,GRY_Offset;

float ACC_X_BUF[AVGtimes],ACC_Y_BUF[AVGtimes],ACC_Z_BUF[AVGtimes];

float time;

float AVGtempX,AVGtempY,AVGtempZ; 

void Cal_TsData() //����׼���Ѽ�����
{ 
	
	u8 i;
	 
	Data_time=TIM4->CNT;
	TIM4->CNT=0;
	
	time=(float)Data_time/1000000;
//--------------���ݽ��е�λת��------------------------//		
  
	ACC_F.X=ACC_RealData.X*ACC_Gain;
  ACC_F.Y=ACC_RealData.Y*ACC_Gain;
	ACC_F.Z=ACC_RealData.Z*ACC_Gain;

  GRY_F.X=GRY_RealData.X*Gyr_Gain;
	GRY_F.Y=GRY_RealData.Y*Gyr_Gain;
	GRY_F.Z=GRY_RealData.Z*Gyr_Gain;

//--------------���ٶ�ƽ��------------------------//	
	ACC_X_BUF[AVG_cnt] = ACC_F.X;
	ACC_Y_BUF[AVG_cnt] = ACC_F.Y;
	ACC_Z_BUF[AVG_cnt] = ACC_F.Z;
	
	for(i=0;i<AVGtimes;i++)
	{
		AVGtempX += ACC_X_BUF[i];
		AVGtempY += ACC_Y_BUF[i];
		AVGtempZ += ACC_Z_BUF[i];
	}
        
	ACC_AVG.X = AVGtempX / AVGtimes;
	ACC_AVG.Y = AVGtempY / AVGtimes;
	ACC_AVG.Z = AVGtempZ / AVGtimes;  // ACC_AVG ���ٶ�ƽ��ֵ
	
	AVGtempX=0;
	AVGtempY=0;
	AVGtempZ=0;
	
	AVG_cnt++;
	if(AVG_cnt>=AVGtimes)	AVG_cnt=0;	
	
}

#define Kp 1.0f   // ��������֧�������������ٶȼ�                     
#define Ki 0.00005f     // ��������֧���ʵ�������ƫ�����ν�                    

float halfT;     // �������ڵ�һ��  

float q0=1, q1=0, q2=0, q3=0;   // ��Ԫ����Ԫ�أ�������Ʒ���  
float exInt=0,eyInt=0,ezInt=0;  // ��������С�������

float_RPY Q_ANGLE;



//��Ԫ��
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{ 
	
	float qq0=0, qq1=0, qq2=0, qq3=0;  

  float norm;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez; 
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

	halfT=(float)time; 	
	
	gx*=0.0174;gy*=0.0174;gz*=0.0174;
  // ��λ����Ԫ�� ȡģ
  norm = sqrtf(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
   // ���Ʒ��������       
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
 // ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;
// ������������������
  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;  
 
  // ������������ǲ���
  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							

  qq0=q0;qq1=q1;qq2=q2;qq3=q3;

  q0 = qq0 + (-qq1*gx - qq2*gy - qq3*gz)*halfT;
  q1 = qq1 + (qq0*gx + qq2*gz - qq3*gy)*halfT;
  q2 = qq2 + (qq0*gy - qq1*gz + qq3*gx)*halfT;
  q3 = qq3 + (qq0*gz + qq1*gy - qq2*gx)*halfT;
	

  // ��������Ԫ��
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  //��Ԫ����ŷ����ת����ʽ
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
  Q_ANGLE.Rool = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 
  Q_ANGLE.Yaw  = atan2(2 * q2 * q1 + 2 * q0 * q3, -2 * q3 * q3 - 2 * q2* q2 + 1)* 57.3; 
}


//New IMU [Bicraze]-----------------------------------------------------------
#define TWO_KP_DEF  (2.0f * 6.0f) // 2 * proportional gain  3.0
//#define TWO_KI_DEF  (2.0f * 0.000005f) // 2 * integral gain  //���˻���ɴ������
#define TWO_KI_DEF  (2.0f * 0.000000f) // 2 * integral gain  //���˻���ɴ������
  
  float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  // integral error terms scaled by Ki


void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ //ע��dt��λΪ100us
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * 0.174;
  gy = gy * 0.174;
  gz = gz * 0.174;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.3;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz)) * 57.3;
  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * 57.3;

//  *pitch = -asin(2 * q1 * q3 + 2 * q0* q2)* 57.3;

//  *roll = atan2(2 * q2 * q3 - 2 * q0 * q1, +2 * q0 * q0 + 2 * q3* q3 - 1)* 57.3; 
//  
//	*yaw  = atan2(2 * q2 * q1 - 2 * q0 * q3, +2 * q1 * q1 + 2 * q0* q0 - 1)* 57.3; 
	
//	*pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
//  *roll   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 
//  *yaw    = atan2(2 * q2 * q1 + 2 * q0 * q3, -2 * q3 * q3 - 2 * q2* q2 + 1)* 57.3; 


}

//-----------------------------------------------------------------------
//�ݶ��½�
#define sampleFreq	2000.0f		// sample frequency in Hz
float Acc_Err;
float Anorm;
float beta = 2;							// proportional gain (Kp)
//---------------------------------------------------------------------------------------------------
	float s0, s1, s2, s3;
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm; 
  float vx, vy, vz;// wx, wy, wz;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float ex, ey, ez;
	
  gx = gx * 0.174;
  gy = gy * 0.174;
  gz = gz * 0.174;
// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;   
  //-------------�õ��������------------------------      
	Anorm = sqrtf(ax*ax + ay*ay + az*az);      
  ax = ax / Anorm;
  ay = ay / Anorm;
  az = az / Anorm;

	vx = 2*((q1 * q3) - (q0 * q2));												
  vy = 2*((q0 * q1) + (q2 * q3));
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;
	
	Acc_Err=(fabs(ex)+fabs(ey)+fabs(ez))*10;

	beta = 2 - Acc_Err;
	
  if(beta<0.2) beta=0.2 ;
  
	//------------------------------------------------
	// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


