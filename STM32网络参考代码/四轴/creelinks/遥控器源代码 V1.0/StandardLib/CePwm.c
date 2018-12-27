/**
  ******************************************************************************
  * @file    CePwm.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CePwm���ļ�
  ******************************************************************************
  * @attention
  *
  *1)����Stm32f103����6��Pwm��Դ���ã����е���Դ֮����ȫ���������ɷֱ��������ڼ�ռ�ձ�
  *2)���ⲿ�ж�Int��ʼ��ʱ������Goio�ڣ�start��ʱ������ΪGPIO_Mode_AF_PP����ʱGpio����PWM���ơ��������stop��ʱ��ȷ��Gpio�ڵĸߵ͵�ƽ������setGpio��resetGpio���ɡ�
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CePwm.h"
#include "CeSystem.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

GPIO_TypeDef*   cePwmGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
#ifdef GPIOE

#endif

#ifdef GPIOF

#endif

#ifdef GPIOG

#endif
    }; 

TIM_TypeDef *   cePwmTimxArray[]   = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8
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
uint8   cePwmTimxInitialStatus[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

const uint8 cePwmTimxAndChannelx[4][16] = {
    0x51,0x52,0x53,0x54,0x00,0x00,0x00,0x00,0x11,0x12,0x13,0x14,0x00,0x00,0x00,0x21,
    0x33,0x34,0x00,0x22,0x31,0x32,0x41,0x42,0x43,0x00,0x23,0x24,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x82,0x83,0x84,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
#ifdef GPIOE

#endif

#ifdef GPIOF

#endif

#ifdef GPIOG

#endif
};//����Pwm��ӦPin����Ӧ��TIMx��channelx
const uint32 cePwmRccApbxTimx[] = {RCC_APB2Periph_TIM1,RCC_APB1Periph_TIM2,RCC_APB1Periph_TIM3,RCC_APB1Periph_TIM4,RCC_APB1Periph_TIM5,RCC_APB1Periph_TIM6,RCC_APB1Periph_TIM7,RCC_APB2Periph_TIM8};
void (*cePwmTimOCxInit[])(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct) = {TIM_OC1Init,TIM_OC2Init,TIM_OC3Init,TIM_OC4Init}; 
void (*cePwmTimOCxPreloadConfig[])(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload) = {TIM_OC1PreloadConfig,TIM_OC2PreloadConfig,TIM_OC3PreloadConfig,TIM_OC4PreloadConfig}; 

void ceCalPwmPrescalerAndPeriod(CePwm* cePwm, uint32 cycleNs, uint32 dutyNs)
{
    uint16 timPrescaler = 0xFFFF;
    uint16 timPeriod = 0xFFFF;
    uint16 timCCRx = 0x7FFF;
    if(cycleNs <= 910222)//���Բ���Ƶ
    {
        timPrescaler = 0;
        timPeriod = (uint16)((cycleNs*72)/1000)+1;
    }else if(cycleNs <= CE_PWM_MAX_CYCLE_NS)//��Ҫ��Ƶ
    {
        timPrescaler = (uint16)(((uint64)cycleNs*72)/((uint64)65536*1000));//
        timPeriod =  (uint16)(((uint64)cycleNs * 72)/((uint64)(timPrescaler+1) * 1000));
    }else
    {
        return;
    }
    timCCRx = (uint16)((uint64)(dutyNs)* (uint64)(timPeriod) / cycleNs);
    cePwm->ceExPwmPar.ceExTimPrescaler = timPrescaler;
    cePwm->ceExPwmPar.ceExTimPeriod = timPeriod;
    cePwm->ceExPwmPar.ceExTimCCRx = timCCRx;
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����CePwmָ�����
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCePwm(CePwm* cePwm)
{
    if (cePwm == (CE_NULL ))
    {
        return CE_STATUS_NULL_POINTER;
    }

    if ((cePwm->ceResource & CE_RES_MARK_PWM) != CE_RES_MARK_PWM) //�����Դ���Ƿ����Ҫ��       
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    if (cePwm->dutyNs < CE_PWM_MIN_CYCLE_NS || cePwm->cycleNs < cePwm->dutyNs)//���ռ�ձ��Ƿ����100%,��ռ�ձ��Ƿ�����Ҫ��
    {
        return CE_STATUS_PAR_ERROR;
    }

    if (cePwm->cycleNs > CE_PWM_MAX_CYCLE_NS || cePwm->cycleNs <= CE_PWM_MIN_CYCLE_NS)//��������Ƿ��ڳ���Χ��
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ʼ��CePwm����
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS cePwm_initial(CePwm* cePwm)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint8 timIndex = 0;
    uint8 channelIndex = 0;

    if(cePwm->cycleNs < CE_PWM_MIN_CYCLE_NS)
    {
        cePwm->cycleNs = CE_PWM_MIN_CYCLE_NS;
    }
    else if(cePwm->cycleNs > CE_PWM_MAX_CYCLE_NS)
    {
        cePwm->cycleNs = CE_PWM_MAX_CYCLE_NS;
    }

    if(cePwm->dutyNs < CE_PWM_MIN_CYCLE_NS)
    {
        cePwm->dutyNs = CE_PWM_MIN_CYCLE_NS;
    }
    else if(cePwm->dutyNs > cePwm->cycleNs)
    {
        cePwm->dutyNs = cePwm->cycleNs;
    }

#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
        
    ceCalPwmPrescalerAndPeriod(cePwm, cePwm->cycleNs, cePwm->dutyNs);
    timIndex= cePwmTimxAndChannelx[(cePwm->ceResource>>4) & 0x0000000F][cePwm->ceResource & 0x0000000F] >> 4 ;
    channelIndex = cePwmTimxAndChannelx[(cePwm->ceResource>>4) & 0x0000000F][cePwm->ceResource & 0x0000000F] & 0x0F;

    cePwm->ceExPwmPar.ceExTimx = cePwmTimxArray[timIndex -1] ;
    cePwm->ceExPwmPar.ceExOldCycle = cePwm->cycleNs;//���ڸ���Pwmʱ����������Ƿ�Ҳ����
    cePwm->ceExPwmPar.ceExGpiox = cePwmGpioxArray[(cePwm->ceResource>>4) & 0x0000000F];
    cePwm->ceExPwmPar.ceExGpioPinx= (uint16)(0x0001) << (cePwm->ceResource & 0x0000000F);
    cePwm->ceExPwmPar.ceExOldDuty = cePwm->dutyNs;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM1 ���ϼ���ʱ����TIMx_CNT < TIMx_CCR*ʱ�������ƽ��Ч������Ϊ��Ч,���¼���ʱ����TIMx_CNT > TIMx_CCR*ʱ�������ƽ��Ч������Ϊ��Ч,TIM_OCMode_PWM2 ��PWM1ģʽ�෴
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = cePwm->ceExPwmPar.ceExTimCCRx;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    if(cePwm->ceExPwmPar.ceExTimx == TIM1) 
    {
        RCC_APB2PeriphClockCmd(cePwmRccApbxTimx[timIndex -1], ENABLE);        //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM1��TIM8��PWM�����Щ���⣬������ʱ�������
    }else if(cePwm->ceExPwmPar.ceExTimx == TIM8)
        RCC_APB2PeriphClockCmd(cePwmRccApbxTimx[timIndex -1], ENABLE);
    else
        RCC_APB1PeriphClockCmd(cePwmRccApbxTimx[timIndex -1], ENABLE);

    if(cePwmTimxInitialStatus[timIndex-1] == 0x00)
    {
        cePwmTimxInitialStatus[timIndex-1] = 0x01;

        TIM_TimeBaseStructure.TIM_Prescaler = cePwm->ceExPwmPar.ceExTimPrescaler;
        TIM_TimeBaseStructure.TIM_Period = cePwm->ceExPwmPar.ceExTimPeriod;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ʱ�ӷָ�
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//�����������ϼ���
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB2PeriphClockCmd(0x00000004 << ((cePwm->ceResource>>4) & 0x0000000F) , ENABLE);
        TIM_DeInit(cePwm->ceExPwmPar.ceExTimx);
        TIM_TimeBaseInit(cePwm->ceExPwmPar.ceExTimx, &TIM_TimeBaseStructure);    
    }
    cePwmTimOCxInit[channelIndex-1](cePwm->ceExPwmPar.ceExTimx, &TIM_OCInitStructure);
    cePwmTimOCxPreloadConfig[channelIndex-1](cePwm->ceExPwmPar.ceExTimx, TIM_OCPreload_Enable);

    //cePwm->ceExPwmPar.ceExTimx->CCER &= (uint16_t)(~(uint16_t)(0x0001 << (channelIndex*4)));


    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼPwm���
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
void cePwm_start(CePwm* cePwm)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8 channelIndex = cePwmTimxAndChannelx[(cePwm->ceResource>>4) & 0x0000000F][cePwm->ceResource & 0x0000000F] & 0x0F;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����Ϊ���������������PWM����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = cePwm->ceExPwmPar.ceExGpioPinx;
    GPIO_Init(cePwm->ceExPwmPar.ceExGpiox, &GPIO_InitStructure);

    TIM_CtrlPWMOutputs(cePwm->ceExPwmPar.ceExTimx, ENABLE);
    TIM_ARRPreloadConfig(cePwm->ceExPwmPar.ceExTimx, ENABLE);            
    TIM_Cmd(cePwm->ceExPwmPar.ceExTimx, ENABLE);    

    cePwm->ceExPwmPar.ceExTimx->CCER |= (uint16_t)(0x0001 << ((channelIndex-1)*4));
}

/**
  * @brief   ����Pwm���
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
void cePwm_updata(CePwm* cePwm)
{
    uint8 channelIndex = 0;
    if(cePwm->ceExPwmPar.ceExOldDuty == cePwm->dutyNs)
        return;
    
    cePwm->ceExPwmPar.ceExTimx->CCER &= (uint16_t)(~(uint16_t)(0x0001 << ((channelIndex-1)*4)));

    if(cePwm->cycleNs < CE_PWM_MIN_CYCLE_NS)
        cePwm->cycleNs = CE_PWM_MIN_CYCLE_NS;
    else if(cePwm->cycleNs > CE_PWM_MAX_CYCLE_NS)
        cePwm->cycleNs = CE_PWM_MAX_CYCLE_NS;

    if(cePwm->dutyNs < CE_PWM_MIN_CYCLE_NS)
        cePwm->dutyNs = CE_PWM_MIN_CYCLE_NS;
    else if(cePwm->dutyNs > cePwm->cycleNs)
        cePwm->dutyNs = cePwm->cycleNs;

#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
    ceCalPwmPrescalerAndPeriod(cePwm, cePwm->cycleNs, cePwm->dutyNs);
        
    if(cePwm->cycleNs != cePwm->ceExPwmPar.ceExOldCycle)
    {
        cePwm->ceExPwmPar.ceExOldCycle = cePwm->cycleNs;
        cePwm->ceExPwmPar.ceExTimx->ARR = cePwm->ceExPwmPar.ceExTimPeriod;
        cePwm->ceExPwmPar.ceExTimx->PSC =cePwm->ceExPwmPar.ceExTimPrescaler;
    }
    channelIndex = cePwmTimxAndChannelx[(cePwm->ceResource>>4) & 0x0000000F][cePwm->ceResource & 0x0000000F] & 0x0F;
    if(channelIndex == 0x01)
        cePwm->ceExPwmPar.ceExTimx->CCR1 = cePwm->ceExPwmPar.ceExTimCCRx;
    else if(channelIndex == 0x02)
        cePwm->ceExPwmPar.ceExTimx->CCR2 = cePwm->ceExPwmPar.ceExTimCCRx;
    else if(channelIndex == 0x03)
        cePwm->ceExPwmPar.ceExTimx->CCR3 = cePwm->ceExPwmPar.ceExTimCCRx;
    else
        cePwm->ceExPwmPar.ceExTimx->CCR4 = cePwm->ceExPwmPar.ceExTimCCRx;

    cePwm->ceExPwmPar.ceExTimx->CCER |= (uint16_t)(0x0001 << ((channelIndex-1)*4));
}

/**
  * @brief   ֹͣPwm���
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
void cePwm_stop(CePwm* cePwm)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8 channelIndex = cePwmTimxAndChannelx[(cePwm->ceResource>>4) & 0x0000000F][cePwm->ceResource & 0x0000000F] & 0x0F;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
    TIM_Cmd(cePwm->ceExPwmPar.ceExTimx, DISABLE);

    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;//���ö�ӦPwm����Ϊ����״̬
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = cePwm->ceExPwmPar.ceExGpioPinx;
    GPIO_Init(cePwm->ceExPwmPar.ceExGpiox, &GPIO_InitStructure);

    /* Disable the Channel 1: Reset the CC1E Bit */
    cePwm->ceExPwmPar.ceExTimx->CCER &= (uint16_t)(~(uint16_t)(0x0001 << (channelIndex*4)));
}

/**
  * @brief   ����Pwm�ڵ����Ϊ�ߵ�ƽ��ע�⣺ֻ����Pwmû�����ʱ���д˲�����
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  None
  */
void cePwm_setBit(CePwm* cePwm)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = cePwm->ceExPwmPar.ceExGpioPinx;
    GPIO_Init(cePwm->ceExPwmPar.ceExGpiox, &GPIO_InitStructure);
    GPIO_SetBits(cePwm->ceExPwmPar.ceExGpiox, cePwm->ceExPwmPar.ceExGpioPinx);
}

/**
  * @brief   ����Pwm�ڵ����Ϊ�͵�ƽ��ע�⣺ֻ����Pwmû�����ʱ���д˲�����
  * @param   cePwm:CePwm���Զ���ָ��
  * @return  None
  */
void cePwm_resetBit(CePwm* cePwm)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCePwm(cePwm));
#endif //__CE_CHECK_PAR__
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = cePwm->ceExPwmPar.ceExGpioPinx;
    GPIO_Init(cePwm->ceExPwmPar.ceExGpiox, &GPIO_InitStructure);
    GPIO_ResetBits(cePwm->ceExPwmPar.ceExGpiox, cePwm->ceExPwmPar.ceExGpioPinx);
}

const CePwmOp cePwmOp = {cePwm_initial, cePwm_start, cePwm_updata, cePwm_stop, cePwm_setBit, cePwm_resetBit};

#ifdef __cplusplus
 }
#endif //__cplusplus
