/**
  ******************************************************************************
  * @file    CeMD.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   �����������ļ�������0~1000������Ϊת��Ϊ0~100%ռ�ձȵ�PWM���
  ******************************************************************************
  * @attention
  *
  *1)����0~1000������ǿ�ȣ������ӦΪ0~100%��ռ�ձ�
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_MD_H__
#define __CE_MD_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_MD_VERSION__ 1                                         /*!< �������ļ��İ汾��*/
#define __CE_MD_NEED_CREELINKS_VERSION__ 1                          /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_MD_NEED_CREELINKS_VERSION__)   /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeMD.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

//#define CE_MD_REVERSE                                             /*!< ���������ĵ�����ͣ�����Pwm�Ƿ���÷��������ʽ������Ч��ƽΪ�͵�ƽ*/

#define CE_MD_MAX_PWM_CYCLE_NS  50000                               /*!< ���������ĵ�����ͣ�����Pwm�����������ڣ���λNs*/
#define CE_MD_MIN_PWM_CYCLE_NS  0                                   /*!< ���������ĵ�����ͣ�����Pwm�������С���ڣ���λNs*/

#define CE_MD_MAX_PWM_DUTY_NS   50000                               /*!< ���������ĵ�����ͣ�����Pwm��������ռ�ձȣ���λNs*/
#define CE_MD_MIN_PWM_DUTY_NS   0                                   /*!< ���������ĵ�����ͣ�����Pwm�������Сռ�ձȣ���λNs*/

/*
 *CeMD���Զ���
 */
typedef struct
{
    CePwm cePwm;                                                    /*!< ģ��ʹ�õ��Ĵ�����Pwm��Դ*/
}CeMD;
/*
 *CeMD��������
 */
typedef struct
{
    CE_STATUS (*initial)(CeMD* ceMD, CE_RESOURCE cePwm);            /*!< @brief CeMDģ���ʼ��
                                                                         @param ceMD:CeMD���Զ���ָ��
                                                                         @param ceXX:CeMDģ��ʹ�õ���Դ��*/
                                                                         
    void      (*setDriverPower)(CeMD* ceMD, uint16 driverPower);    /*!< @brief ����Pwm������ǿ�ȣ�0~1000����Ӧռ�ձ�Ϊ0%~100%
                                                                         @param ceMD:CeMD���Զ���ָ��
                                                                         @param driverPower:Pwm������ǿ�ȣ�0~1000����Ӧռ�ձ�Ϊ0%~100%*/
}CeMDOp;
/*
 *CeMD��������ʵ��
 */
extern const CeMDOp ceMDOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_MD_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_MD_H__

