/**
  ******************************************************************************
  * @file    CeSystem.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeSystem��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)Debugģʽ�£�printf��������������ȥ������ɺ󣬲ŷ���
  *2)�뾡�����ڸ����ж��е���printf����
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeSystem.h"
#include "CeTicker.h"
#include "CeDebug.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
const char ceStatusString[9][30] = {{"CE_STATUS_SUCCESS"},{"CE_STATUS_FAILE"},{"CE_STATUS_RESOURCE_ERROR"},{"CE_STATUS_INITIAL_FALSE"},{"CE_STATUS_NULL_POINTER"},{"CE_STATUS_MALLOC_FALSE"},{"CE_STATUS_PAR_ERROR"},{"CE_STATUS_OUT_TIME"},{"UnknowError"}};

#define CE_PRIORITY_GROUP_CONFIG NVIC_PriorityGroup_2//����STM32F103���ж���

CeSystem ceSystem;

#ifdef __CE_CHECK_PAR__

#ifdef  USE_FULL_ASSERT
extern void assert_failed(uint8_t* file, uint32_t line);
#endif //USE_FULL_ASSERT

/**
  * @brief   ����ʧ�ܵĴ���������ʹ�õ�UART���д����ʽ��ͨ��UART���������Ϣ����λ�����մ˴�����Ϣ��ͨ������Ϣ���Զ�λ������ʧ�ܵĴ�����
  * @param   file:����ʧ�ܵ��ļ�·��
  * @param   line:����ʧ�ܵĴ������ļ��е�����
  * @param   ceStatus:����ʧ�ܵ�״̬��
  * @return  ״ָ̬ʾ��
  */
void ce_assert_failed(uint8_t* file, uint32_t line, CE_STATUS ceStatus)
{
    if (ceStatus == CE_STATUS_SUCCESS)
    {
        return;
    }
    while (1)
    {
        ceDebugOp.printf("There is an error occurred:\n\tFile:%s\nLine:%d\nCE_STATUS:%s\n\n", file, line, ceSystemOp.getErrorMsg(ceStatus));
        ceSystemOp.delayMs(1000);
    }
}
#endif //__CE_CHECK_PAR__

/**
  * @brief    Creelinksƽ̨�жϳ�ʼ����������ϵͳ����ǰ�����
  * @param    None
  * @return  ״ָ̬ʾ��
  */
CE_STATUS ceSystem_GolbalInterruptInitial(void)
{
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(CE_PRIORITY_GROUP_CONFIG);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   Systemʹ�õ��Ķ�ʱ���жϻص�����
  * @param   pAddPar:��ָ��
  */
void ceSystem_callBackTickTimer(void* pAddPar)
{
    ceSystem.tickLoopNow++;
    ceTickerOp.callBySystem();
}

/**
  * @brief   ��ʼ��1Us��ȷ��ʱ��
  * @param   None
  * @return  ״ָ̬ʾ��
  */
CE_STATUS ceSystem_initialTickTimer()
{
    ceSystem.ceTimer.callBack = ceSystem_callBackTickTimer;
    ceSystem.ceTimer.ceResource = CE_SYSTEM_DELAY_USE_TIMX;
    ceSystem.ceTimer.intervalNs = 5000000;//��Ҫ�����嶨ʱ��������ж����ڵ�ʱ��������λns
    ceSystem.ceTimer.pAddPar = &ceSystem;
    ceSystem.tickLoopNow = 0;
    ceTimerOp.initial(&(ceSystem.ceTimer));
    ceTimerOp.start(&(ceSystem.ceTimer));
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ȡϵͳ�ӿ�����������������ʱ��,����1Us
  * @param   None
  * @return  ��ʱ�����е���ʱ��
  */
uint64 ceSystem_getSystemTickUs(void)
{
    uint32 maxCnt,nowCnt;
    maxCnt = (uint32)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint32)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x01)); 
    return ((uint64)maxCnt*ceSystem.tickLoopNow+nowCnt)/72;
}

/**
  * @brief   ��ȡϵͳ�ӿ�����������������ʱ��,����1Ms
  * @param   None
  * @return  ��ʱ�����е���ʱ��
  */
uint64 ceSystem_getSystemTickMs(void)
{
    #ifdef __CE_USE_RTOS__
    return OSTimeGet();
    #else
    return ceSystem_getSystemTickUs()/1000;
    #endif
}

/**
  * @brief   Creelinksƽ̨��ʱ����
  * @param   us:��Ҫ��ʱ��ʱ�䣬��λ΢��
  * @return  None
  */
void ceSystem_delayUs(uint32 us)
{
    if(us == 0)
    {
        return;
    }else
    {
    uint64 maxCnt,nowCnt;    
    uint64 usTemp;    
      usTemp = ceSystem_getSystemTickUs();
    maxCnt = (uint64)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 

    while((maxCnt*ceSystem.tickLoopNow+nowCnt)/72 - usTemp < us)
        nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 
    }
}

/**
  * @brief   Creelinksƽ̨��ʱ����
  * @param   ns:��Ҫ��ʱ��ʱ�䣬��λ����
  * @return  None
  */
void ceSystem_delayNs(uint32 ns)
{
   ceSystem_delayUs(ns / 1000);
}


/**
  * @brief   Creelinksƽ̨��ʱ����
  * @param   us:��Ҫ��ʱ��ʱ�䣬��λ����
  * @return  None
  */
void ceSystem_delayMs(uint32 ms)
{
#ifdef __CE_USE_RTOS__
    if (OSRunning == 0x00)
        for (; ms > 0; ms--)
            ceSystem_delayUs(1000);
    else
        if (ms == 0)
            ceSystem_taskSchedule();//ֱ�ӽ����������
        else
            OSTimeDly (ms);
#else
    uint64 usTemp = ceSystem_getSystemTickUs();
    uint64 maxCnt,nowCnt;
    maxCnt = (uint64)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 

    while((maxCnt*ceSystem.tickLoopNow+nowCnt)/72 - usTemp < (uint64)ms*1000)
        nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 
#endif
}

/**
  * @brief   ���ݷ��صĴ����룬�õ�char*���͵Ĵ���������Ϣ
  * @param   ceStatus:״̬��
  * @return  ���ַ�����ʽ����״̬��
  */
const char* ceSystem_getErrorMsg(CE_STATUS ceStatus)
{
    return ceStatusString[(uint8)ceStatus];
}


/**
  * @brief   Creelinksƽ̨��ʼ����������ϵͳ����ǰ�����
  * @param   None
  * @return  ״ָ̬ʾ��
  */
CE_STATUS ceSystem_initial(void)
{
    ceSystem_GolbalInterruptInitial();
    ceSystem_initialTickTimer();

    #ifdef __CE_USE_RTOS__
    OSInit();
    #endif
    return CE_STATUS_SUCCESS;
}

const CeSystemOp ceSystemOp = {ceSystem_initial,
                               ceSystem_delayNs,ceSystem_delayUs, ceSystem_delayMs, ceSystem_getSystemTickUs, ceSystem_getSystemTickMs, 
                               ceSystem_getErrorMsg};
#ifdef __cplusplus
 }
#endif //__cplusplus
