/**
  ******************************************************************************
  * @file    CeBtnx1.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBtnx1ģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBtnx1.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  Gpio��ʽ��������ʱ�Ļص�����
  * @param  pAddPar:CeBtnx1���Զ���
  * @return None
  */
void ceBtnx1_callBackTick(void* pAddPar)
{
    CeBtnx1* ceBtnx1 = (CeBtnx1*)pAddPar;
    if(ceGpioOp.getBit(&ceBtnx1->ceGpio) == 0x00)//�������������
    {
        if((ceBtnx1->btnStatus & 0x01) == 0x00)
        {
            if(ceBtnx1->callBackPressEvent != CE_NULL)
            {
                ceBtnx1->callBackPressEvent();
            }
            ceBtnx1->btnStatus |= 0x01;              //��ֹ�ظ���Ӧ
        }
    }
    else
    {
        ceBtnx1->btnStatus &= (~0x01);
    }
}

/**
  * @brief  �ⲿ�ж�Int��ʽ��������ʱ�Ļص�����
  * @return None
  */
void ceBtnx1_callBackInt(void* pAddPar)
{
    if(((CeBtnx1*)pAddPar)->callBackPressEvent != CE_NULL)
    {
        ((CeBtnx1*)pAddPar)->callBackPressEvent();
    }
}

/**
  * @brief  CeBtnx1ģ��ʹ��Gpio������ɳ�ʼ��
  * @param  ceBtnx1:CeBtnx1���Զ���
  * @param  ceGpio:CeBtnx1ģ��ʹ�õ���Դ��
  * @param  callBackPressEvent:��������ʱ�Ļص�����������Ҫ�ص���CE_NULL����
  * @return ϵͳ״̬��
  */
CE_STATUS ceBtnx1_initialByGpio(CeBtnx1* ceBtnx1, CE_RESOURCE ceGpio, void (*callBackPressEvent)(void))
{
    ceBtnx1->btnStatus = 0x00;
    ceBtnx1->callBackPressEvent = callBackPressEvent;
    ceBtnx1->ceGpio.ceResource = ceGpio;
    ceBtnx1->ceGpio.ceGpioMode = CE_GPIO_MODE_IPU;
    ceGpioOp.initial(&(ceBtnx1->ceGpio));

    if (ceBtnx1->callBackPressEvent != CE_NULL)
    {
        ceBtnx1->ceTicker.ID = ceGpio;
        ceBtnx1->ceTicker.callBack = ceBtnx1_callBackTick;
        ceBtnx1->ceTicker.intervalMs = 100;
        ceBtnx1->ceTicker.pAddPar = ceBtnx1;

        ceTickerOp.registerTicker(&(ceBtnx1->ceTicker));
        ceTickerOp.start(&(ceBtnx1->ceTicker));
    }
    else if(ceBtnx1->ceTicker.ID == ceGpio)
    {
        ceTickerOp.unRegister(&(ceBtnx1->ceTicker));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  CeBtnx1ģ��ʹ���ⲿ�ж�Int����ɳ�ʼ��
  * @param  ceBtnx1:CeBtnx1���Զ���
  * @param  ceInt:CeBtnx1ģ��ʹ�õ���Դ��
  * @param  callBackPressEvent:��������ʱ�Ļص�����������Ҫ�ص���CE_NULL����
  * @return ϵͳ״̬��
  */
CE_STATUS ceBtnx1_initialByInt(CeBtnx1* ceBtnx1, CE_RESOURCE ceInt, void (*callBackPressEvent)(void))
{
    ceBtnx1->btnStatus = 0x80;
    ceBtnx1->callBackPressEvent = callBackPressEvent;
    ceBtnx1->ceInt.ceResource = ceInt;
    ceBtnx1->ceInt.callBack = ceBtnx1_callBackInt;
    ceBtnx1->ceInt.ceIntMode = CE_INT_MODE_TRIGGER_FALLING;//��̬�ߵ�ƽ���½��ش���
    ceBtnx1->ceInt.pAddPar = ceBtnx1;
    ceIntOp.initial(&(ceBtnx1->ceInt));
    if (ceBtnx1->callBackPressEvent != CE_NULL)
    {
        ceIntOp.start(&(ceBtnx1->ceInt));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  ��ȡCeBtnx1״̬������1�����Ѱ��£�����0����δ����
  * @param  ceBtnx1:CeBtnx1���Զ���
  * @return Gpio�ĵ�ƽֵ��0x01��0x00
  */
uint8 ceBtnx1_getStatus(CeBtnx1* ceBtnx1)
{
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        return (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x00)? 0x01:0x00;
    }
    else
    {
        return (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x00)? 0x01:0x00;
    }
}

/**
  * @brief  �ȴ���������(������ⲿ�ж�Int��ʽ��ʼ���������ʹ�ô˷���)
  * @param  ceBtnx1:CeBtnx1���Զ���
  * @param  outTimeMs:�ȴ��ĳ�ʱʱ�䣬Ms
  * @return ϵͳ״̬�룬CE_STATUS_SUCCESS��CE_STATUS_OUT_TIME
  */
CE_STATUS ceBtnx1_waitForPressDown(CeBtnx1* ceBtnx1, uint32 outTimeMs)
{
    uint32 temp = ceSystemOp.getSystemTickMs();
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        while (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x01)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
#ifdef CE_USE_RTOS          //������ڲ���ϵͳ�����£����������л�
            ceSystemOp.delayMs(0);
#endif
        };
    }
    else
    {
        while (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x01)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
#ifdef CE_USE_RTOS          //������ڲ���ϵͳ�����£����������л�
        ceSystemOp.delayMs(0);
#endif
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  �ȴ���������
  * @param  ceBtnx1:CeBtnx1���Զ���
  * @param  outTimeMs:�ȴ��ĳ�ʱʱ�䣬Ms
  * @return ϵͳ״̬�룬CE_STATUS_SUCCESS��CE_STATUS_OUT_TIME
  */
CE_STATUS ceBtnx1_waitForPressUp(CeBtnx1* ceBtnx1, uint32 outTimeMs)
{
    uint32 temp = ceSystemOp.getSystemTickMs();
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        while (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x00)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
    }
    else
    {
        while (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x00)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
    }
    return CE_STATUS_SUCCESS;
}

const CeBtnx1OpBase ceBtnx1Op = {ceBtnx1_initialByGpio, ceBtnx1_initialByInt, ceBtnx1_getStatus, ceBtnx1_waitForPressDown, ceBtnx1_waitForPressUp};

#ifdef __cplusplus
 }
#endif //__cplusplus
