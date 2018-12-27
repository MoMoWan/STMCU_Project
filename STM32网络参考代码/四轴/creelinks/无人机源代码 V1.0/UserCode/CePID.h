/**
  ******************************************************************************
  * @file    CePID.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ���˻�PID������������ģ��
  ******************************************************************************
  * @attention
  *
  *1)���뵱ǰ���˻���̬��Pitch/Roll/Yaw�����߶ȡ��������š�
  *2)����ĸ����������ǿ�ȣ�0~1000.
  *3)Ĭ��PID����������cePID_initial�����г�ʼ������ֵ
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PID_H__
#define __CE_PID_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#include "CeFilter.h"

#define  CE_PID_MIN_DRIVER_POWER    20      /*!< ����ά�ֵ��������ת����С��������0~1000��*/
#define  CE_PID_MAX_DRIVER_POWER    1000    /*!< �������ɳ��ܵ������������0~1000*/

/**
  * @brief  �ṹ�壬�ĸ�����������������ţ���0~10000���ɰ�����ת��Ϊ�����ת��
  */
typedef struct
{
    int16 driverPower0;                     /*!< M0�������ģ����������ǿ�ȣ�0~1000*/
    int16 driverPower1;                     /*!< M1�������ģ����������ǿ�ȣ�0~1000*/
    int16 driverPower2;                     /*!< M2�������ģ����������ǿ�ȣ�0~1000*/
    int16 driverPower3;                     /*!< M3�������ģ����������ǿ�ȣ�0~1000*/
}CeDrivePower;

/**
  * @brief  �ṹ�壬PID���������Զ��󣬴���PID
  */
typedef struct  
{   
    //Pitch Pid������
    fp32 outPitchP;                         /*!< Pitch�ǿ��ƣ��⻷Kp��*/
    fp32 outPitchI;                         /*!< Pitch�ǿ��ƣ��⻷Ki��*/
    fp32 outPitchD;
    fp32 outPitchError;                     /*!< Pitch�ǿ��ƣ��⻷�Ƕ�����ۻ���*/
    fp32 outPidPitch;                       /*!< Pitch�ǿ��ƣ��⻷PID���������*/
    fp32 lastOutPitch;
    fp32 inPitchP;                          /*!< Pitch�ǿ��ƣ��ڻ�Kp��*/
    fp32 inPitchI;                          /*!< Pitch�ǿ��ƣ��ڻ�Ki��*/
    fp32 inPitchD;                          /*!< Pitch�ǿ��ƣ��ڻ�Kd��*/
    fp32 inPitchError;                      /*!< Pitch�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 inPidPitch;                        /*!< Pitch�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 lastInPitchGyrY;                   /*!< Pitch�ǿ��ƣ���һ���ڻ��Ƕ���*/
    //Roll Pid������
    fp32 outRollP;                          /*!< Roll�ǿ��ƣ��⻷Kp��*/
    fp32 outRollI;                          /*!< Roll�ǿ��ƣ��⻷Ki��*/
    fp32 outRollD;
    fp32 outRollError;                      /*!< Roll�ǿ��ƣ��⻷�Ƕ�����ۻ���*/
    fp32 outPidRoll;                        /*!< Roll�ǿ��ƣ��⻷PID���������*/
    fp32 lastOutRoll;
    fp32 inRollP;                           /*!< Roll�ǿ��ƣ��ڻ�Kp��*/
    fp32 inRollI;                           /*!< Roll�ǿ��ƣ��ڻ�Ki��*/
    fp32 inRollD;                           /*!< Roll�ǿ��ƣ��ڻ�Kd��*/
    fp32 inRollError;                       /*!< Roll�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 inPidRoll;                         /*!< Roll�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 lastInRollGyrX;                    /*!< Roll�ǿ��ƣ���һ���ڻ��Ƕ���*/
    //Yaw Pid���������δʹ�õشţ��ʽ��õ����ڻ����⻷����Ԥ��
    fp32 outYawP;                          /*!< Yaw�ǿ��ƣ��⻷Kp��*/
    fp32 outYawI;                          /*!< Yaw�ǿ��ƣ��⻷Ki��*/
    fp32 outYawD;
    fp32 outYawError;                      /*!< Yaw�ǿ��ƣ��⻷�Ƕ�����ۻ���*/
    fp32 outPidYaw;                        /*!< Yaw�ǿ��ƣ��⻷PID���������*/
    fp32 lastOutYaw;
    fp32 inYawP;                           /*!< Yaw�ǿ��ƣ��ڻ�Kp��*/
    fp32 inYawI;                           /*!< Yaw�ǿ��ƣ��ڻ�Ki��*/
    fp32 inYawD;                           /*!< Yaw�ǿ��ƣ��ڻ�Kd��*/
    fp32 inYawError;                       /*!< Yaw�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 inPidYaw;                         /*!< Yaw�ǿ��ƣ��ڻ��Ƕ�����ۻ���*/
    fp32 lastInYawGyrZ;                     /*!< Yaw�ǿ��ƣ���һ���ڻ��Ƕ���*/

    fp32 altBase;
    fp32 altKp;
    fp32 altKi;
    fp32 altKd;
    fp32 altPid;
    fp32 altError;
    fp32 lastAltError;

    CeDrivePower drivePower;                /*!< �ĸ����������ǿ��*/
    CeDrivePower drivePowerZero;            /*!< �ĸ����������ǿ����㣬���ھ�ȷУ������������񡢲��������εȵ��µĵ��ת�ٲ���ͬ*/
    CePackageSend* cePackageSend;           /*!< ���ݴ��������ʹ�õĽṹ��*/
    CePackageRecv* cePackageRecv;           /*!< ���ݲ��������ʹ�õĽṹ��*/
}CePID;

/**
  * @brief  �ṹ�壬PID��������������
  */
typedef struct  
{
    void            (*initial)(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv);                         /*!< @brief  CePID������ģ���ʼ��
                                                                                                                         @param  cePackageSend:���ݴ��������ʹ�õĽṹ��
                                                                                                                         @param  cePackageRecv:���ݲ��������ʹ�õĽṹ��*/

    CeDrivePower*   (*calculate)(CeAcc* ceNowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngles, CeAngles* ceHopeAngles,fp32 dtS);  /*!< 
                                                                                                                         @brief  ���ݵ�ǰ���˻����ٶȡ����ٶȡ���̬�ǡ�������̬�ǣ�ֱ�Ӽ�����ĸ����Ӧ�е�����ǿ��
                                                                                                                         @param  ceNowAcc:��ǰ���˻�������ٶ����ݣ���λG
                                                                                                                         @param  ceNowGyr:��ǰ���˻�������ٶ����ݣ���λ��/s
                                                                                                                         @param  ceNowAngles:��ǰ���˻���̬������
                                                                                                                         @param  ceHopeAngles:�������˻����ڵ���̬������
                                                                                                                         @param  dtS:����ִ�����ڣ���λS*/
}CePIDOp;

/*
 *CePID��������ʵ��
 */
extern const CePIDOp cePIDOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_PID_H__

