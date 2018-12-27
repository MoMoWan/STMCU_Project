/**
  ******************************************************************************
  * @file    CeTicker.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeTicker��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)��ʱ����С���Ϊ1ms,���ȸ��ݲ�ͬ�Ĵ�����ƽ̨����ͬ�����STM32F103��ԼΪ����2us��
  *2)����޲���ϵͳ��ƽ̨��ע��Ķ�ʱ������������ϵͳ�ж��ڣ�����в���ϵͳ��ƽ̨����������һ�������ȼ����߳��ڣ��ʾ������ڶ�ʱ�������ڽ��к�ʱ������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTicker.h"
#include "CeTask.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

CeTicker* ceTickerList = CE_NULL;
uint32 ceTickerLastMs = 0;

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����û����ݵ�ceTickerָ���Ƿ���ȷ
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_INITIAL_FALSE��CE_STATUS_NULL_POINTER��CE_STATUS_PAR_ERROR
  */
CE_STATUS ceCheckCeTicker(CeTicker* ceTicker)
{
    if (ceTicker == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceTicker->intervalMs == 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if (ceTicker->callBack == CE_NULL)
    {
        return CE_STATUS_INITIAL_FALSE;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ϵͳ��ÿ1ms����һ�εĺ���
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  None
  */
CE_STATUS ceTicker_callBySystem()
{
    uint64 ttt = ceSystemOp.getSystemTickMs();
    
    if(ttt - ceTickerLastMs < CE_TICKER_CALL_TIME_MS) return CE_STATUS_SUCCESS;
    ceTickerLastMs = ceSystemOp.getSystemTickMs();
    if(ceTickerList != CE_NULL)
    {
        CeTicker* ceTickerTemp = ceTickerList;
        while(ceTickerTemp != CE_NULL)
        {
            if(ceTickerTemp->ceExTickerPar.isRunning == 0x01)
            {
                ceTickerTemp->ceExTickerPar.nowTick += CE_TICKER_CALL_TIME_MS;
                if(ceTickerTemp->ceExTickerPar.nowTick >= ceTickerTemp->intervalMs)
                {
                    ceTickerTemp->callBack(ceTickerTemp->pAddPar);
                    ceTickerTemp->ceExTickerPar.nowTick = 0x0000;
                }
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ע��һ����ʱ��
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_register(CeTicker* ceTicker)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
#endif //__CE_CHECK_PAR__

    ceTicker->nextCeTicker = CE_NULL;
    ceTicker->ceExTickerPar.isRunning = 0x00;

    if (ceTickerList == CE_NULL)
    {
        ceTickerList = ceTicker;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;
        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                break;
            }
            if (ceTickerTemp->nextCeTicker == CE_NULL)
            {
                ceTickerTemp->nextCeTicker = ceTicker;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼһ����ʱ������
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_start(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;

#ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
#endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                ceTickerTemp->ceExTickerPar.isRunning = 0x01;
                ceTickerTemp->ceExTickerPar.nowTick = 0x0000;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ֹͣһ����ʱ������
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS cTicker_stop(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;
        #ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
        #endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                ceTickerTemp->ceExTickerPar.isRunning = 0x00;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ɾ��һ����ʱ������
  * @param   ceTicker:ceTickert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_unRegister(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerBefore = CE_NULL;
        CeTicker* ceTickerTemp = ceTickerList;
        #ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
        #endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                if (ceTickerBefore == CE_NULL)
                {
                    ceTickerList = CE_NULL;
                } else
                {
                    if (ceTickerTemp->nextCeTicker == CE_NULL)
                    {
                        ceTickerBefore->nextCeTicker = CE_NULL;
                    } else
                    {
                        ceTickerBefore->nextCeTicker = ceTickerTemp->nextCeTicker;
                    }
                }
                break;
            }
            ceTickerBefore = ceTickerTemp;
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

const CeTickerOp ceTickerOp = {ceTicker_register, ceTicker_start, cTicker_stop, ceTicker_unRegister,ceTicker_callBySystem};

#ifdef __cplusplus
}
#endif //__cplusplus
