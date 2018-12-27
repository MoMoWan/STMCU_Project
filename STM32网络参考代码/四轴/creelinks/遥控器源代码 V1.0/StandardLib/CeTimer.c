/**
  ******************************************************************************
  * @file    CeTimer.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeTimer��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTimer.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

const uint8 ceTimerIntPriority[] = {0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31};/*!< ��ʱ����Ӧ����ռʽ���ȼ�����Ӧ���ȼ���ǰ��bitΪ��ռ���ȼ�������bitΪ��Ӧ���ȼ�*/

#define CE_TIMER_NOT_USE        0x00        /*!< Timer��Դδ��ʹ��*/
#define CE_TIMER_USE_CCP        0x01        /*!< Timer��Դ����Ccp*/
#define CE_TIMER_USE_TIMER      0x02        /*!< Timer��Դ����Timer*/

uint8 ceTimer2_useStatus = 0x00;            /*!< ��ʱ��Timer2��Դ��ʹ��״̬��ֵ����CE_TIMER_NOT_USE��ʾδ��ʹ��
                                                                             ֵ����CE_TIMER_USE_CCP��ʾ����Ccp
                                                                             ֵ����CE_TIMER_USE_TIMER��ʾ����Timer*/

const uint32 CE_TIMER_MAX_CYCLE_NS=(uint32)59652323555;         /*!< ��ʱ��֧�ֵ����ʱ���*/
const uint32 CE_TIMER_MIN_CYCLE_NS=(uint32)28;                  /*!< ��ʱ��֧�ֵ����ʱ���*/
const uint32 CE_TIMER_MIN_DIVIDE_NS=(uint32)14;                 /*!< ��ʱ��֧�ֵ���С��ʱ����*/


TIM_TypeDef *   ceTimerTimxArray[]   = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8
#ifdef TIM9
    ,TIM9
#endif

#ifdef TIM10
    ,TIM10
#endif

#ifdef TIM11
    ,TIM11
#endif

#ifdef TIM12
    ,TIM12
#endif

#ifdef TIM13
    ,TIM13
#endif

#ifdef TIM14
    ,TIM14
#endif
};
uint32 ceTimerRccApbxTimx[] = {RCC_APB2Periph_TIM1,RCC_APB1Periph_TIM2,RCC_APB1Periph_TIM3,RCC_APB1Periph_TIM4,RCC_APB1Periph_TIM5,RCC_APB1Periph_TIM6,RCC_APB1Periph_TIM7,RCC_APB2Periph_TIM8
#ifdef TIM9
,RCC_APB2Periph_TIM9
#endif

#ifdef TIM10
,RCC_APB2Periph_TIM10
#endif

#ifdef TIM11
,RCC_APB2Periph_TIM11
#endif

#ifdef TIM12
,RCC_APB1Periph_TIM12
#endif

#ifdef TIM13
,RCC_APB1Periph_TIM13
#endif

#ifdef TIM14
,RCC_APB1Periph_TIM14
#endif
};
IRQn_Type ceTimerIRQChannel[] = {
TIM1_UP_IRQn     ,TIM2_IRQn,TIM3_IRQn,TIM4_IRQn,TIM5_IRQn,TIM6_IRQn,TIM7_IRQn,TIM8_UP_IRQn};
CeTimer* ceTimerList[] = {CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL
#ifdef TIM9
, CE_NULL
#endif

#ifdef TIM10
, CE_NULL
#endif

#ifdef TIM11
, CE_NULL
#endif

#ifdef TIM12
, CE_NULL
#endif

#ifdef TIM13
, CE_NULL
#endif

#ifdef TIM14
, CE_NULL
#endif

#ifdef TIM15
, CE_NULL
#endif  

#ifdef TIM16
, CE_NULL
#endif  

#ifdef TIM17
, CE_NULL
#endif  
    };

#ifdef __CE_USE_CCP__
extern void ceCcp_OverflowInterrupt(void);
#endif // __CE_USE_CCP__

/**
  * @brief   ����Da��������֮ǰ��ת��������ƻ�TIM��ʱ���ļĴ���ֵ
  * @param   ceDaTimBaseData
  * @param   intervalNs
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceCalTimerPrescalerAndPeriod(CeTimer* ceTimer, uint64 intervalNs)
{
    uint16 timPrescaler = 0xFFFF;
    uint16 timPeriod = 0xFFFF;
      
    if(intervalNs <= 910222)//���Բ���Ƶ
    {
        timPrescaler = 0;
        timPeriod = (uint16)((intervalNs*72)/1000)+1;
    }else //��Ҫ��Ƶ
    {
        timPrescaler = (uint16)(((uint64)intervalNs*72)/((uint64)65536000));//
        timPeriod =  (uint16)(((uint64)intervalNs * 72)/((uint64)(timPrescaler+1)*1000));
    }
    ceTimer->ceExTimerPar.ceExTimPrescaler = timPrescaler;
    ceTimer->ceExTimerPar.ceExTimPeriod = timPeriod;
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����û����ݵ�ceTimerָ���Ƿ���ȷ
  * @param   ceTimer:ceTimert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_INITIAL_FALSE��CE_STATUS_NULL_POINTER��CE_STATUS_PAR_ERROR
  */
CE_STATUS ceCheckCeTimer(CeTimer* ceTimer)
{
    if (ceTimer == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceTimer->intervalNs == 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if (ceTimer->callBack == CE_NULL)
    {
        return CE_STATUS_INITIAL_FALSE;
    }
    if((ceTimer->ceResource & CE_RES_MARK_TIMER) != CE_RES_MARK_TIMER)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   TIM1�ж���ں���
  * @param   None
  * @return  None
  */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[0]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[0]->callBack(ceTimerList[0]->pAddPar);
        }        
    }
}


/**
  * @brief   TIM2�ж���ں���(CCP����ֵ�ﵽ�趨ֵ����жϺ���)
  * @param   None
  * @return  None
  *//*
void TIM2_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimer2_useStatus == CE_TIMER_USE_TIMER)
        {
            if (ceTimerList[1]->ceExTimerPar.ceExIsStart == 0x01)
            {
                if (ceTimerList[1]->callBack != CE_NULL)
                {
                    ceTimerList[1]->callBack(ceTimerList[1]->pAddPar);
                }
            }
        }
#ifdef __CE_USE_CCP__
        else if(ceTimer2_useStatus == CE_TIMER_USE_CCP)
        {
            ceCcp_OverflowInterrupt();
        }
#endif // __CE_USE_CCP__
    }
}*/

/**
  * @brief   TIM3�ж���ں���
  * @param   None
  * @return  None
  */
void TIM3_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[2]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[2]->callBack(ceTimerList[2]->pAddPar);
        }
    }
}


/**
  * @brief   TIM4�ж���ں���
  * @param   None
  * @return  None
*/
void TIM4_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM4, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[3]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[3]->callBack(ceTimerList[3]->pAddPar);
        }
    }
}  



/**
  * @brief   TIM5�ж���ں���
  * @param   None
  * @return  None
  */ 
void TIM5_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM5, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
       if (ceTimerList[4]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[4]->callBack(ceTimerList[4]->pAddPar);
        }
    }
} 

/**
  * @brief   TIM6�ж���ں���
  * @param   None
  * @return  None
  */
void TIM6_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[5]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[5]->callBack(ceTimerList[5]->pAddPar);
        }
    }
}


/**
  * @brief   TIM7�ж���ں���
  * @param   None
  * @return  None
  */
void TIM7_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM7, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM7, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[6]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[6]->callBack(ceTimerList[6]->pAddPar);
        }
    }
}




/**
  * @brief   TIM8�ж���ں���
  * @param   None
  * @return  None
  */
void TIM8_UP_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM8, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM8, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        if (ceTimerList[7]->ceExTimerPar.ceExIsStart == 0x01)
        {
            ceTimerList[7]->callBack(ceTimerList[7]->pAddPar);
        }
    }
}

/**
  * @brief   ��ϵͳ���õĶ�ʱ����ʼ������
  * @param   ceTimer:ceTimert���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_INITIAL_FALSE��CE_STATUS_SUCCESS
  */
CE_STATUS ceTimer_initial(CeTimer* ceTimer)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    uint8 timIndex;
    
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTimer(ceTimer));
#endif //__CE_CHECK_PAR__
    
    ceCalTimerPrescalerAndPeriod(ceTimer, ceTimer->intervalNs);
    TIM_TimeBaseStructure.TIM_Prescaler = ceTimer->ceExTimerPar.ceExTimPrescaler;
    TIM_TimeBaseStructure.TIM_Period = ceTimer->ceExTimerPar.ceExTimPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//�����������ϼ���

    timIndex = 0x0000000F & ceTimer->ceResource;
    ceTimerList[timIndex] = ceTimer;
    ceTimer->ceExTimerPar.ceExTimx = ceTimerTimxArray[timIndex];
    if(timIndex+1 == 0x01 && timIndex+1 == 0x08)
    {
        RCC_APB2PeriphClockCmd(ceTimerRccApbxTimx[timIndex],ENABLE);
    }else if(timIndex+1 == 0x02)
    {
        ceTimer2_useStatus = CE_TIMER_USE_TIMER;////0x02ΪTimer��Դ����Timer
        RCC_APB1PeriphClockCmd(ceTimerRccApbxTimx[timIndex],ENABLE);
    }else
    {
        RCC_APB1PeriphClockCmd(ceTimerRccApbxTimx[timIndex],ENABLE);
    }
    NVIC_InitStructure.NVIC_IRQChannel = ceTimerIRQChannel[timIndex];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ceTimerIntPriority[timIndex] >>4;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = ceTimerIntPriority[timIndex] & 0x0F;

    ceTimer->ceExTimerPar.ceExIsStart = 0x00;//��ʼ����߾��ȶ�ʱ��Timer���ڷǹ���״̬

    TIM_DeInit(ceTimer->ceExTimerPar.ceExTimx);//��λTIMx��ʱ��
    TIM_TimeBaseInit(ceTimer->ceExTimerPar.ceExTimx, &TIM_TimeBaseStructure);
    
    TIM_ClearFlag(ceTimer->ceExTimerPar.ceExTimx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_SetCounter(ceTimer->ceExTimerPar.ceExTimx, 0);
    TIM_ITConfig(ceTimer->ceExTimerPar.ceExTimx, TIM_IT_Update, DISABLE);//DISABLE TIMx Update interrupt TIMx����жϽ�

    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ����һ����ʱ���������
  * @param   ceTimer:��ʱ��ָ��
  * @return  None
  */
void  ceTimer_upData(CeTimer* ceTimer)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTimer(ceTimer));
#endif //__CE_CHECK_PAR__
    if (ceTimer->ceExTimerPar.ceExIsStart == 0x01)//����ѿ�ʼ״̬���ȹرն�ʱ���ٽ��в���
    {
        TIM_Cmd(ceTimer->ceExTimerPar.ceExTimx, DISABLE);
        TIM_ITConfig(ceTimer->ceExTimerPar.ceExTimx, TIM_IT_Update, DISABLE);
        TIM_ClearFlag(ceTimer->ceExTimerPar.ceExTimx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
    }

    ceCalTimerPrescalerAndPeriod(ceTimer, ceTimer->intervalNs);
    /* Set the Autoreload value */
    ceTimer->ceExTimerPar.ceExTimx->ARR = ceTimer->ceExTimerPar.ceExTimPeriod;
    /* Set the Prescaler value */
    ceTimer->ceExTimerPar.ceExTimx->PSC = ceTimer->ceExTimerPar.ceExTimPrescaler;

    if (ceTimer->ceExTimerPar.ceExIsStart == 0x01)//���֮ǰ�ǿ�ʼ״̬��Ҫ������ָ�״̬
    {
        TIM_SetCounter(ceTimer->ceExTimerPar.ceExTimx, 0);
        TIM_ClearFlag(ceTimer->ceExTimerPar.ceExTimx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
        TIM_ITConfig(ceTimer->ceExTimerPar.ceExTimx, TIM_IT_Update, ENABLE);
        TIM_Cmd(ceTimer->ceExTimerPar.ceExTimx, ENABLE);
    }
}

/**
  * @brief   ��ʼһ����ʱ������
  * @param   ceTimer:ceTimert���Զ���ָ��
  * @return  None
  */
void ceTimer_start(CeTimer* ceTimer)
{
    ceTimer->ceExTimerPar.ceExIsStart = 0x01;
    TIM_SetCounter(ceTimer->ceExTimerPar.ceExTimx, 0);
    TIM_ClearFlag(ceTimer->ceExTimerPar.ceExTimx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
    TIM_ITConfig(ceTimer->ceExTimerPar.ceExTimx, TIM_IT_Update, ENABLE);
    TIM_Cmd(ceTimer->ceExTimerPar.ceExTimx, ENABLE);
}

/**
  * @brief   ֹͣһ����ʱ������
  * @param   ceTimer:ceTimert���Զ���ָ��
  * @return  None
  */
void ceTimer_stop(CeTimer* ceTimer)
{
    ceTimer->ceExTimerPar.ceExIsStart = 0x00;
    TIM_Cmd(ceTimer->ceExTimerPar.ceExTimx, DISABLE);
    TIM_ITConfig(ceTimer->ceExTimerPar.ceExTimx, TIM_IT_Update, DISABLE);
    TIM_ClearFlag(ceTimer->ceExTimerPar.ceExTimx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
}

/**
  * @brief ��ȡ��ʱ�������������ֵ��Ĭ�Ͼ�Ϊ���ϼ��������Ϊ���¼�����������Ҫת��Ϊ���ϼ���
  * @param ceTimer:��ʱ��ָ��
  * @return  ��ǰ��ʱ���������ֵ����ƵҲ��������
  */
uint32 ceTimer_getTimerMaxCnt(CeTimer* ceTimer)   
{
    return ((uint32)(ceTimer->ceExTimerPar.ceExTimPeriod) * (ceTimer->ceExTimerPar.ceExTimPrescaler +1));
}

/**
  * @brief ��ȡ��ʱ����������ֵ��Ĭ�Ͼ�Ϊ���ϼ��������Ϊ���¼�����������Ҫת��Ϊ���ϼ���
  * @param ceTimer:��ʱ��ָ��
  * @param isStopInt:0x01:��ȡ����ֹͣ��ʱ���жϣ�0x00:��ȡʱ����ֹͣ��ʱ���ж�
  * @return  ��ǰ��ʱ������ֵ����ƵҲ��������
  */
uint32 ceTimer_getTimreNowCnt(CeTimer* ceTimer,uint8 isStopInt)
{
    if(isStopInt == 0x01)
    {
        uint32 temp;
        ceTimer->ceExTimerPar.ceExTimx->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
        temp = ((uint32)(ceTimer->ceExTimerPar.ceExTimx->CNT) * (ceTimer->ceExTimerPar.ceExTimx->PSC + 1));
        ceTimer->ceExTimerPar.ceExTimx->CR1 |= TIM_CR1_CEN;
        return temp;
    }else
    {
        return ((uint32)(ceTimer->ceExTimerPar.ceExTimx->CNT) * (ceTimer->ceExTimerPar.ceExTimx->PSC + 1));
    }
}
const CeTimerOp ceTimerOp = {ceTimer_initial, ceTimer_start, ceTimer_upData, ceTimer_stop,ceTimer_getTimerMaxCnt,ceTimer_getTimreNowCnt};

#ifdef __cplusplus
 }
#endif //__cplusplus
