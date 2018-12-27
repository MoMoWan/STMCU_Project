/**
  ******************************************************************************
  * @file    CeBeepNSrc.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBeepNSrcģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBeepNSrc.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  CeBeepNSrcģ���ʼ��
  * @param  ceBeepNSrc:CeBeepNSrc���Զ���
  * @param  ceXX:CeBeepNSrcģ��ʹ�õ���Դ��
  * @return ϵͳ״̬��
  */
CE_STATUS ceBeepNSrc_initial(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE ceGpio)
{
    ceBeepNSrc->ceGpio.ceResource = ceGpio;
    ceBeepNSrc->ceGpio.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBeepNSrc->ceGpio));
    ceGpioOp.resetBit(&(ceBeepNSrc->ceGpio));
    return CE_STATUS_SUCCESS;
}

CE_STATUS ceBeepNSrc_initialByPwm(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE cePwm)
{
        return CE_STATUS_SUCCESS;
}

/**
  * @brief CeBeepNSrc��������������Դ�������߳̽��ᱻ������ֱ���������
  * @param ceBeepNSrc:CeBeepNSrc���Զ���ָ��
  * @param durationMs:����ʱ�䣬��λ����
  * @param sleepMs:ֹͣ����ʱ��
  * @param beepTimes:��������
  */
void ceBeepNSrc_say(CeBeepNSrc* ceBeepNSrc, uint16 sayMs,uint16 sleepMs, uint8 beepTimes)
{
    int loop,i;
    for(i=0;i<beepTimes;i++)
    {
         ceTaskOp.inCriticalSection();
         for(loop = 0;loop<sayMs*5;loop++)//5Khz   
         {
             CE_SET_GPIO_BIT(&(ceBeepNSrc->ceGpio));
             ceSystemOp.delayUs(100);
             CE_RESET_GPIO_BIT(&(ceBeepNSrc->ceGpio));    
             ceSystemOp.delayUs(100);
         }
         ceTaskOp.outCriticalSection();
         ceSystemOp.delayMs(sleepMs);
    }
}


/**
  * @brief  CeBeepNSrcģ�����������
  */
const CeBeepNSrcOp ceBeepNSrcOp = {ceBeepNSrc_initial,ceBeepNSrc_initialByPwm,ceBeepNSrc_say};

#ifdef __cplusplus
 }
#endif //__cplusplus
