
#include "stm32f10x.h"

typedef struct int16_xyz{
				vs16 X;
				vs16 Y;
				vs16 Z;} int16_XYZ;  //�з���16λxyz

typedef struct float_xyz{
				float X;
				float Y;
				float Z;} float_XYZ; //������xyz


typedef struct float_angle{
				float Rool;
				float Pitch;
				float Yaw;} float_RPY; //������ŷ����


typedef struct PID{

float kp,ki,kd,IOUT,D,DOUT,IMAX,Err,PreErr,desired,integ,I_Max,deriv,outP,outI,outD,P,I;
}PID; //PID�����ṹ��
extern float PitchOutput,RollOutput,YawOutput;
extern int16_XYZ ACC_RealData;
extern int16_XYZ GRY_RealData;
extern u8 TxDate[32];
extern u8 RxData[8];
extern float_XYZ ACC_F,GRY_F;
extern float_RPY Q_ANGLE;

extern float_XYZ ACC_AVG;

extern u8 TIM4over;

extern u16 Data_time;

extern u8 FLy_count;
extern u8 FLY_Enable;

extern int16_XYZ ACC_Offset,GRY_Offset;

extern int16_XYZ MAG_XYZ;									
									
extern float_XYZ EXP_ANGLE;

extern u16 dir_time;

extern float MAG_Offset,MAG_angle ;

extern u8 MAG_Ofs_OK;

typedef struct   //���ͽṹ
{
	float Acc_data[3];
	float F_GRY_data[3];
	float F_Cal_data[2];

} RF_data;

extern float Acc_Err;
extern float thr;

typedef struct   //���սṹ
{
  u8 ii1 ;
	u8 rc_thr;
	s8 rc_pit;
	s8 rc_rol;
	u8 ii2 ;
	s8 rc_yaw;
	u8 x55 ;
  u8 xaa ;
} RC_TPRY;

typedef union//���߽���
{
	RC_TPRY   RCdata; 	
  u8        RxData[8];
} RC;

extern RC RCun;

extern u16 Peace; //��ֹУ׼ ;
