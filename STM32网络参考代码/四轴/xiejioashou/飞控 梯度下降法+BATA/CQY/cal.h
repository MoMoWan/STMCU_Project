#include "stm32f10x.h"
void Cal_TsData(void); //����׼���Ѽ�����
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);








