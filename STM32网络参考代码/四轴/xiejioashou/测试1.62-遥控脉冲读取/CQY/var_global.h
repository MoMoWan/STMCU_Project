
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

float P,POUT,I,IOUT,D,DOUT,IMAX,SetPoint,NowPoint,LastError,PrerError;

                  }PID; //PID�����ṹ��

extern int16_XYZ ACC_RealData;
extern int16_XYZ GRY_RealData;
extern u8 TxDate[4];
extern u8 RxData[4];
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

extern float PWM_X,PWM_Y ;

extern float MAG_Offset,MAG_angle ;

extern u8 MAG_Ofs_OK;


