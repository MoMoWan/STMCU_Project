/**
  ******************************************************************************
  * @file    CeTimer.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ������ͷ�ļ����������о�ȷ��ʱ����صĲ���
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TIMER_H__
#define __CE_TIMER_H__

#include "CeMcu.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  �ṹ�壬Timer����������Լ���
  */
typedef struct
{
    CE_RESOURCE ceResource;
    uint32      intervalNs;                                 /*!< ��ʱ����ʱ���*/
    void*       pAddPar;                                    /*!< ��ָ��*/
    void        (*callBack)(void* pAddPar);                 /*!< ���ﶨʱʱ�����Ҫִ�еĺ���*/

    CeExTimerPar   ceExTimerPar;                            /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeTimer;

/**
  * @brief  �ṹ�壬Timer������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeTimer* ceTimer);              /*!< @brief ��ϵͳ���õĶ�ʱ����ʼ������
                                                                 @param ceTimer:��ʱ��ָ��*/

    void        (*start)(CeTimer* ceTimer);                 /*!< @brief ��ʼһ����ʱ����
                                                                 @param ceTimer:��ʱ��ָ��*/

    void        (*upData)(CeTimer* ceTimer);                /*!< @brief ����һ����ʱ���������
                                                                 @param ceTimer:��ʱ��ָ��*/

    void        (*stop)(CeTimer* ceTimer);                  /*!< @brief ֹͣһ����ʱ����
                                                                 @param ceTimer:��ʱ��ָ��*/

    uint32      (*getTimerMaxCnt)(CeTimer* ceTimer);        /*!< @brief ��ȡ��ʱ�������������ֵ
                                                                 @param ceTimer:��ʱ��ָ��*/

    uint32      (*getTimerNowCnt)(CeTimer* ceTimer,uint8 isStopInt); /*!< 
                                                                 @brief ��ȡ��ʱ����������ֵ
                                                                 @param ceTimer:��ʱ��ָ��
                                                                 @param isStopInt:0x01:��ȡ����ֹͣ��ʱ���жϣ�0x00:��ȡʱ����ֹͣ��ʱ���ж�*/
}CeTimerOp;
extern const CeTimerOp ceTimerOp;                           /*!< ������Timer�صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TIMER_H__
