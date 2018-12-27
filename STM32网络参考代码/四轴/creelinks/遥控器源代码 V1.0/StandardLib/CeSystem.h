/**
  ******************************************************************************
  * @file    CeSystem.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨�����й�ϵͳ�ĺ���
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_SYSTEM_H__
#define __CE_SYSTEM_H__

#include "CeMcu.h"
#include "CeTimer.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief ���ڼ���ʱ�仨�ѵ����Զ���
  */
typedef struct
{
     uint32 nowTickUs;
}CeTimeCost;


typedef struct
{
    CeTimer ceTimer;
    uint32 tickLoopNow;
}CeSystem;
/**
  * @brief  ϵͳ������صĺ������Ͻṹ�����
  */
typedef struct
{
    CE_STATUS   (*initial)(void);       /*!< @brief CreeLinks������ʼ������Ҫ��main�ʼʱ����*/

    void        (*delayNs)(uint32 ns);  /*!< @brief ΢����ʱ������1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param ns:�趨����ʱʱ�䣬��Χ0~60129542144ns����������14ns*/

    void        (*delayUs)(uint32 us);  /*!< @brief ΢����ʱ������1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param us:�趨��΢����ʱʱ�䣬��Χ0~59652323us����������14us*/

    void        (*delayMs)(uint32 ms);  /*!< @brief ��������ʱ������1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param ms:�趨�ĺ�����ʱʱ�䣬��Χ0~4294967296ms����������1us*/

    uint64      (*getSystemTickUs)(void);/*!<@brief ��ȡϵͳ�ӿ�����������������ʱ��,����1Us
                                             @return ��ȡϵͳ�ӿ��������ڵ�����ʱ�䣬��λUs*/

    uint64      (*getSystemTickMs)(void);/*!<@brief ��ȡϵͳ�ӿ�����������������ʱ��,����1Ms
                                             @return ��ȡϵͳ�ӿ��������ڵ�����ʱ�䣬��λMs*/

    const char* (*getErrorMsg)(CE_STATUS ceStatus);/*!<
                                             @brief ���ݷ��صĴ����룬�õ�char*���͵Ĵ���������Ϣ
                                             @param ceStatus:״̬��
                                             @return ���ַ�����ʽ����״̬��*/
}CeSystemOp;
extern const CeSystemOp ceSystemOp; /*!< ������ϵͳƽ̨��صĲ��������ӡ����ʱ����ʼ����*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_SYSTEM_H__
