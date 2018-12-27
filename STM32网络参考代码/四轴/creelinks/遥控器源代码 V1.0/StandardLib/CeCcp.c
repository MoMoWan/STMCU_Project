/**
  ******************************************************************************
  * @file    CeCcp.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeCcp��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)�Ƴ���������ص����ǽ����ڶ�ʱ��������ж�֮�ϵģ������ڻص���ִ�й������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeCcp.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#define CE_CCP_R5_PREEMPTION_PRIORITY   3       /*!< ��ԴR5��Ӧ��ʱ������жϵ���ռʽ���ȼ�*/
#define CE_CCP_R5_SUB_PRIORITY          1       /*!< ��ԴR5��Ӧ��ʱ������жϵ���Ӧʽ���ȼ�*/

typedef struct
{
    CeCcp* tim2Ccp;
} CeCcpCallBackList;

CeCcpCallBackList ceCcpCallBackList = { CE_NULL};/*!< �ݴ�Ccp������*/
//#ifdef __CE_USE_TIMER__
//extern CeTimerList ceTimerList;//R34Timer���ڲ��߾��ȶ�ʱ����R5ACGPW��CCPʹ�õ���ͬһ����ʱ��TIMx��Դ
//#endif


extern uint8 ceTimer2_useStatus;
void ceCcp_OverflowInterrupt(void)
{
    if (ceCcpCallBackList.tim2Ccp != CE_NULL)
    {
        if (ceCcpCallBackList.tim2Ccp->callBackReachCntVal != CE_NULL)
        {
            ceCcpCallBackList.tim2Ccp->callBackReachCntVal(ceCcpCallBackList.tim2Ccp->pAddPar);
            
        }
        ceCcpCallBackList.tim2Ccp->ceExCcpPar.ceExOutCcpCnt++;
    }
}

/**
 * @brief   ����ֵ�ﵽ�趨ֵ����жϺ���
 * @param   ceCcp:ceCcp���Զ���ָ��
 * @return  None
 */
void TIM2_IRQHandler()
{
    if (TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        if (ceCcpCallBackList.tim2Ccp != CE_NULL)
        {
            if (ceCcpCallBackList.tim2Ccp->callBackReachCntVal != CE_NULL)
            {
                ceCcpCallBackList.tim2Ccp->callBackReachCntVal(ceCcpCallBackList.tim2Ccp->pAddPar);
                
            }
            ceCcpCallBackList.tim2Ccp->ceExCcpPar.ceExOutCcpCnt++;
        }
    }
}


#ifdef __CE_CHECK_PAR__
/**
  * @brief   ��֤Ccp���Զ������ȷ��
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeCcp(CeCcp* ceCcp)
{
    if (ceCcp == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if ((ceCcp->ceResource & CE_RES_MARK_CCP) != CE_RES_MARK_CCP)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ʼ��CCP������
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCcp_initial(CeCcp* ceCcp)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__

    ceCcp->ceExCcpPar.ceExOutCcpCnt = 0;
    if (ceCcp->ceResource == PA0ACGIP)
    {
#ifdef __CE_USE_TIMER__
        ceTimer2_useStatus = 0x01;//0x01ΪTimer��Դ����Ccp
#endif
        ceCcp->ceExCcpPar.ceExTIMx = TIM2;
        ceCcpCallBackList.tim2Ccp = ceCcp;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //Give Up if Not Working!
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CE_CCP_R5_PREEMPTION_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = CE_CCP_R5_SUB_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        TIM_DeInit(ceCcp->ceExCcpPar.ceExTIMx);
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_Period = (uint16) (ceCcp->ceCntVal);
        TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(ceCcp->ceExCcpPar.ceExTIMx, &TIM_TimeBaseStructure);
        TIM_ClearFlag(ceCcp->ceExCcpPar.ceExTIMx, TIM_FLAG_Update);
        TIM_ETRClockMode2Config(ceCcp->ceExCcpPar.ceExTIMx, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
        TIM_SetCounter(ceCcp->ceExCcpPar.ceExTIMx, 0);
        TIM_ITConfig(ceCcp->ceExCcpPar.ceExTIMx, TIM_IT_Update, ENABLE);
    }
    else
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼCCP����
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceCcp_start(CeCcp* ceCcp)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__
    TIM_SetCounter(ceCcp->ceExCcpPar.ceExTIMx, 0);
    TIM_Cmd(ceCcp->ceExCcpPar.ceExTIMx, ENABLE);
}

/**
  * @brief   ֹͣCCP����
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceCcp_stop(CeCcp* ceCcp)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__
    TIM_Cmd(ceCcp->ceExCcpPar.ceExTIMx, DISABLE);
}

/**
  * @brief   ��õ�ǰCCP������ֵ����ֵһ��С�ڵ���ceCntVal
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return   ��ȡ���μ������ڵļ���ֵ
  */
uint32 ceCcp_getNowCnt(CeCcp* ceCcp)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__
    return ceCcp->ceExCcpPar.ceExTIMx->CNT;
}

/**
  * @brief   ��ôӿ�ʼ�����𣬵�����һ���ļ���ֵ
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  �ӿ�ʼ������ĿǰΪֹ�ܵļ���ֵ
  */
uint32 ceCcp_getAllCnt(CeCcp* ceCcp)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__
    return ceCcp->ceExCcpPar.ceExOutCcpCnt * ceCcp->ceCntVal + ceCcp->ceExCcpPar.ceExTIMx->CNT;
}

/**
  * @brief   �����������0��ʼ���¼���
  * @param   ceCcp:ceCcp���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceCcp_clearCnt(CeCcp* ceCcp)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeCcp(ceCcp));
#endif //__CE_CHECK_PAR__
    ceCcp_stop(ceCcp);
    ceCcp->ceExCcpPar.ceExTIMx->CNT = 0;
    ceCcp->ceExCcpPar.ceExOutCcpCnt = 0;
    ceCcp_start(ceCcp);
}

const CeCcpOp ceCcpOp = {ceCcp_initial, ceCcp_start, ceCcp_stop, ceCcp_getNowCnt, ceCcp_getAllCnt, ceCcp_clearCnt};

#ifdef __cplusplus
 }
#endif //__cplusplus
