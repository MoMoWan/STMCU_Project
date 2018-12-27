/**
  ******************************************************************************
  * @file    CeTask.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeTask��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)���ǰ��̨�����޲���ϵͳ���ƽ̨��Ϊ����������ѭ��mainTask���������û��뱣֤mainTask���������ڵ��ö�����������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTask.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef __CE_USE_RTOS__
#define CE_TASK_PRIORITY_START_BASE 5                               /*!< uCOSII �ɴ���64����������0��1��2��3/4����Ϊ�������������5��ʼ��HH���������ʼID���ɴ���11��HH������*/
#define CE_TASK_PRIORITY_H_BASE     5 + OS_LOWEST_PRIO / 5          /*!< H���������ʼID���ɴ���11��H������*/
#define CE_TASK_PRIORITY_M_BASE     5 + OS_LOWEST_PRIO * 2 / 5      /*!< M�������ʼID���ɴ���11��M������*/
#define CE_TASK_PRIORITY_L_BASE     5 + OS_LOWEST_PRIO * 3 / 5      /*!< L���������ʼID���ɴ���11��H������*/
#define CE_TASK_PRIORITY_LL_BASE    5 + OS_LOWEST_PRIO * 4 / 5      /*!< LL���������ʼID���ɴ���11��H������*/
#define CE_TASK_PRIORITY_END_BASE   OS_LOWEST_PRIO                  /*!< uCOSII �ɴ���64����������63��62��61��60����Ϊ�����������ﵽ59��ֹ*/
uint8 ceTask_priorityHHIndex    =   CE_TASK_PRIORITY_START_BASE;
uint8 ceTask_priorityHIndex     =   CE_TASK_PRIORITY_H_BASE;
uint8 ceTask_priorityMIndex     =   CE_TASK_PRIORITY_M_BASE;
uint8 ceTask_priorityLIndex     =   CE_TASK_PRIORITY_L_BASE;
uint8 ceTask_priorityLLIndex    =   CE_TASK_PRIORITY_LL_BASE;

#define CE_MAIN_TASK_STACK_SIZE  2048
CE_STK  CE_MAIN_TASK_STACK[CE_MAIN_TASK_STACK_SIZE];
CeTask ceTaskRtos;
#endif //CE_USE_RTOS

CeTask* ceTaskList = CE_NULL;
uint16 ceTask_criticalTimes = 0;//�����ٽ�εĴ���

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ���ceTask��������ȷ��
  * @param   ceTask:ceTask���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeTask(CeTask* ceTask)
{
    if (ceTask == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceTask->callBack == CE_NULL)
    {
        return CE_STATUS_INITIAL_FALSE;
    }
#ifdef __CE_USE_RTOS__
    if (ceTask->taskPriority > 4)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if (ceTask->isNewThread != 0x01 && ceTask->isNewThread != 0x00)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if ( ceTask->taskPriority > CE_TASK_PRIORITY_LL || ceTask->taskStackBuf == CE_NULL || ceTask->taskStackBufSize == 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
#endif //CE_USE_RTOS
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__


#ifdef __CE_USE_RTOS__
/**
  * @brief   ���ڲ��ܱ�֤�û��ṩ���������Ƿ�Ϊ��ѭ�����������������ѭ���������û���������
  * @param   pAddPar:ceTask���Զ���ָ��
  */
void ceTask_uCOSIITaskPlam(void* pAddPar)
{
    while (1)
    {
        ((CeTask*)(pAddPar))->callBack(((CeTask*)(pAddPar))->pAddPar);
    };
}
#endif //CE_USE_RTOS


/**
  * @brief  ϵͳ��ѭ���е��õĺ���
  * @param  ceTask:ceTask���Զ���ָ��
  */
CE_STATUS ceTask_mainTask(void)
{
    #ifdef __CE_USE_RTOS__
    OS_CPU_SysTickInit();
    //OSStatInit();             //��ʼ��ͳ������
    OSStart();                  //main����ִ�е�����󣬾Ͳ�������ִ���ˣ�Ҳ����˵����������ִֹͣ���ˡ�
    #else 
    if (ceTaskList != CE_NULL)
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (ceTaskTemp != CE_NULL)
        {
            if (ceTaskTemp->ceExTaskPar.isRunning == 0x01)
            {
                ceTaskTemp->callBack(ceTaskTemp->pAddPar);
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    #endif
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ע��һ��CeTask
  * @param   ceTask:ceTask���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceTask_register(CeTask* ceTask)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTask(ceTask));
#endif //__CE_CHECK_PAR__
    ceTask->nextCeTask = CE_NULL;
    ceTask->ceExTaskPar.isRunning = 0x00;

#ifdef __CE_USE_RTOS__
    switch (ceTask->taskPriority)//�Զ������Ż����ķ��䣬��ע�����������ȼ�����
    {
    case CE_TASK_PRIORITY_HH:
        if (ceTask_priorityHHIndex != CE_TASK_PRIORITY_H_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHHIndex;
            ceTask_priorityHHIndex++;
        }
        else if(ceTask_priorityHIndex != CE_TASK_PRIORITY_M_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHIndex;
            ceTask_priorityHIndex++;
        }else if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_H:
        if (ceTask_priorityHIndex != CE_TASK_PRIORITY_M_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHIndex;
            ceTask_priorityHIndex++;
        }
        else if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_M:
        if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_L:
        if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_LL:
        if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    default:
        break;
    }
    if (ceTask->isNewThread == 0x01)//���ģ���������Ҫ�½�һ������
    {
        uint8_t os_err;
        //ceDebugOp.printf("CeTask Create Task,ID=%u,ceExTaskPriority=%u.\n", ceTask->ID, ((ceTask->ceExTaskPar).ceExTaskPriority));//�����ã�ע�⣬��Ҫ����ע��Debugǰ����������
        OSTaskCreateExt(
            ceTask_uCOSIITaskPlam,
            ceTask,
            (OS_STK*)(ceTask->taskStackBuf + ceTask->taskStackBufSize - 1),
            ceTask->ceExTaskPar.ceExTaskPriority,
            ceTask->ceExTaskPar.ceExTaskPriority,
            (OS_STK *)(ceTask->taskStackBuf),
            ceTask->taskStackBufSize,
            (void *)0,
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
        OSTaskNameSet(ceTask->ceExTaskPar.ceExTaskPriority, (uint8*)(ceTask->taskName),&os_err);//�趨���������
    }
    else
    {
        if (ceTaskList == CE_NULL)
        {
            ceTaskList = ceTask;
        }
        else
        {
            CeTask* ceTaskTemp = ceTaskList;
            while (1)
            {
                if (ceTask->ID == ceTaskTemp->ID)
                {
                    break;
                }
                if (ceTaskTemp->nextCeTask == CE_NULL)
                {
                    ceTaskTemp->nextCeTask = ceTask;
                    break;
                }
                ceTaskTemp = ceTaskTemp->nextCeTask;
            }
        }
    }
#else
    if (ceTaskList == CE_NULL)
    {
        ceTaskList = ceTask;
    }
    else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                break;
            }
            if (ceTaskTemp->nextCeTask == CE_NULL)
            {
                ceTaskTemp->nextCeTask = ceTask;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
#endif //CE_USE_RTOS
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼһ��CeTask
  * @param   ceTask:ceTask���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_start(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                ceTaskTemp->ceExTaskPar.isRunning = 0x01;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ֹͣһ��CeTask
  * @param   ceTask:ceTask���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_stop(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                ceTaskTemp->ceExTaskPar.isRunning = 0x00;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ɾ��һ��CeTask
  * @param   ceTask:ceTask���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_unRegister(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskBefore = CE_NULL;
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                if (ceTaskBefore == CE_NULL)
                {
                    ceTaskList = CE_NULL;
                } else
                {
                    if (ceTaskTemp->nextCeTask == CE_NULL)
                    {
                        ceTaskBefore->nextCeTask = CE_NULL;
                    } else
                    {
                        ceTaskBefore->nextCeTask = ceTaskTemp->nextCeTask;
                    }
                }
                break;
            }
            ceTaskBefore = ceTaskTemp;
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}


/**
  * @brief   �����ٽ�δ��룬�˺��ֹ�����л�
  * @param   None
  * @return  ״ָ̬ʾ��
  */
void ceTask_inCriticalSection(void)
{
    ceTask_criticalTimes++;
    //��RTOS�������˺���Ϊ��
#ifdef __CE_USE_RTOS__
    //OS_ENTER_CRITICAL();
#endif
}

/**
  * @brief   �����ٽ�δ��룬�˺������л�������������
  * @param   None
  */
void ceTask_outCriticalSection(void)
{

    if (ceTask_criticalTimes > 0)
    {
        ceTask_criticalTimes--;
    }
    if (ceTask_criticalTimes == 0)
    {
        #ifdef __CE_USE_RTOS__
        //OS_EXIT_CRITICAL();
        #else
        //��RTOS������Ϊ���ж�
        #endif
    }
}
/**
  * @brief ��֤�Ƿ����ٽ�״̬
  * @return ����0x01:�������ٽ�ף�0x00��δ�����ٽ����
  */
uint8 ceTask_getCriticalStatus(void) 
{
    if(ceTask_criticalTimes > 0)
        return 0x01;
    else return 0x00;
}


/**
  * @brief   ֪ͨ����ϵͳ�������������
  * @param   None
  * @retur   ״ָ̬ʾ��
  */
void ceTask_taskSchedule(void)
{
    //��RTOS�������˺���Ϊ��
#ifdef __CE_USE_RTOS__
    ceSystemOp.delayMs(1);
#endif
}



const CeTaskOp ceTaskOp = {ceTask_mainTask,ceTask_register, ceTask_start, ceTask_stop, ceTask_unRegister,ceTask_inCriticalSection,ceTask_outCriticalSection,ceTask_getCriticalStatus,ceTask_taskSchedule};

#ifdef __cplusplus
 }
#endif //__cplusplus
