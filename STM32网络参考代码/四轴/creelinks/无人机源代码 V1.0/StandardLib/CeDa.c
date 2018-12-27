/**
  ******************************************************************************
  * @file    CeDa.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ����STM32F103RET6������ƽ̨��CeDa��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)Daת����ɺ�Ļص����ǻ���DMA��������жϻص��ģ������ڻص���ִ�к�ʱ����
  *2)Daִ�л�����ЩBug����ʱ���ڽ����������εĴ��벻Ҫɾ����
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeDa.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#define CE_DA_PA4_PREEMPTION_PRIORITY   2                               /*!< ��ԴR10��ӦDMA�жϵ���ռʽ���ȼ�*/
#define CE_DA_PA4_SUB_PRIORITY          1                               /*!< ��ԴR10��ӦDMA�жϵ���Ӧʽ���ȼ�*/

#define CE_DA_PA5_PREEMPTION_PRIORITY   2                               /*!< ��ԴR12��ӦDMA�жϵ���ռʽ���ȼ�*/
#define CE_DA_PA5_SUB_PRIORITY          1                               /*!< ��ԴR12��ӦDMA�жϵ���Ӧʽ���ȼ�*/

#define CE_DAC1_Address                 0x40007408                      /*!< Da1ͨ����DMA��Ӧ��ַ*/
#define CE_DAC2_Address                 0x40007414                      /*!< Da2ͨ����DMA��Ӧ��ַ*/
#define CE_DA_MID2_INTERVAL_NS          (uint32)7456540444//65536       /*!< ��������ת��ʱ�����м�ֵ����ʱ��65536��Ƶ*/
#define CE_DA_MID1_INTERVAL_NS          (uint32)910222//8192            /*!< ��������ת��ʱ�����м�ֵ����ʱ��8192��Ƶ*/

typedef struct
{
    uint16 timPrescaler;
    uint16 timPeriod;
} CeDaTimBaseData;

typedef struct
{
    CeDa* ceDa1;
    CeDa* ceDa2;
} CeDaList;

CeDaList ceDaList = {CE_NULL, CE_NULL};
const uint16 ceDaTemp[1] = { 0x000};           /*!< ��ʼ��DMAͨ��ʱ�ĵ�ַ*/

/**
  * @brief   ����Da��������֮ǰ��ת��������ƻ�TIM��ʱ���ļĴ���ֵ
  * @param   ceDaTimBaseData
  * @param   intervalNs
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceCalDaPrescalerAndPeriod(CeDaTimBaseData* ceDaTimBaseData, uint32 intervalNs)
{
    uint16 timPrescaler = 0xFFFF;
    uint16 timPeriod = 0xFFFF;
    if (intervalNs > CE_DA_MAX_INTERVAL_NS)
    {
        intervalNs = CE_DA_MAX_INTERVAL_NS;
    }
    if (intervalNs < CE_DA_MIN_INTERVAL_NS)
    {
        intervalNs = CE_DA_MIN_INTERVAL_NS;
    }
    if (intervalNs >= CE_DA_MID2_INTERVAL_NS)
    {
        timPrescaler = 0xFFFF;
    } 
    else if (intervalNs >= CE_DA_MID1_INTERVAL_NS)
    {
        timPrescaler = 0x1FFF;
    } 
    else
    {
        timPrescaler = 0x0000;
    }
    timPeriod = ((uint64) intervalNs * 72) / (1000 * (timPrescaler + 1));
    if (timPeriod < 7)
    {
        timPeriod = 7;
    }
    ceDaTimBaseData->timPrescaler = timPrescaler;
    ceDaTimBaseData->timPeriod = timPeriod;
}

/**
  * @brief   DMA2 CH3ͨ�����ݴ�������жϣ���ӦDa1
  * @return  None
  */
void DMA2_Channel3_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_FLAG_TC3) != RESET)
    {
        DMA_ClearFlag(DMA2_FLAG_GL3);//����жϱ�־
        TIM_Cmd(TIM6, DISABLE);//ʧ�ܶ�ʱ��
        DMA_Cmd(DMA2_Channel3, DISABLE);//ʧ��DACͨ����DMA
        DAC_DMACmd(DAC_Channel_1, DISABLE);//ʧ��DACͨ����DMA
        DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, DISABLE);//ʧ�ܷ���DMAͨ���ж�
        DAC_Cmd(DAC_Channel_1, DISABLE);//ʧ��DACͨ��
        ceDaList.ceDa1->ceExDaPar.ceExIsConvertFinish = 0x01;//����DaΪδ����״̬
        if (ceDaList.ceDa1->callBackConvertFinish != CE_NULL)
        {
            ceDaList.ceDa1->callBackConvertFinish(ceDaList.ceDa1->pAddPar);
        }
    }
}

/**
  * @brief   DMA2 CH4ͨ�����ݴ�������жϣ���ӦDa2
  * @return  None
  */
void DMA2_Channel4_5_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_FLAG_TC4) != RESET)
    {
        TIM_Cmd(TIM7, DISABLE);//ʧ�ܶ�ʱ��
        DMA_ClearFlag(DMA2_FLAG_TC4);//����жϱ�־
        DMA_Cmd(DMA2_Channel4, DISABLE);//ʧ��DACͨ����DMA
        DAC_DMACmd(DAC_Channel_2, DISABLE);//ʧ��DACͨ����DMA
        DMA_ITConfig(DMA2_Channel4, DMA_IT_TC, DISABLE);//ʧ�ܷ���DMAͨ���ж�
        DAC_Cmd(DAC_Channel_2, DISABLE);//ʧ��DACͨ��
        ceDaList.ceDa2->ceExDaPar.ceExIsConvertFinish = 0x01;//����DaΪδ����״̬
        if (ceDaList.ceDa2->callBackConvertFinish != CE_NULL)
        {
            ceDaList.ceDa2->callBackConvertFinish(ceDaList.ceDa2->pAddPar);
        }
    }
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����Da����ָ���Ƿ���ȷ
  * @param   ceDa:Da���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeDa(CeDa* ceDa)
{
    if (ceDa == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    // if(ceDa->callBackConvertFinish == CE_NULL)//�����û���ʹ��DA�Ļص�
    // {
    //     return CE_STATUS_NULL_POINTER;
    // }
    if (ceDa->ceResource & CE_RES_MARK_DA != CE_RES_MARK_DA)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ʼ��һ��Da
  * @param   ceDa:Da���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceDa_initial(CeDa* ceDa)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    CeDaTimBaseData ceDaTimBaseData;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    ceDa->ceExDaPar.ceExIsConvertFinish = 0x01;
    if (ceDa->ceResource == PA4ADGI)
    {
        ceDa->ceExDaPar.ceExDMAChannel = DMA2_Channel3;
        ceDa->ceExDaPar.ceExTIMx = TIM6;
        ceDa->ceExDaPar.ceExDAC_Channel_x = DAC_Channel_1;
        ceDa->ceExDaPar.ceExGpiox = GPIOA;
        ceDa->ceExDaPar.ceExGpioPinx = GPIO_Pin_4;
        ceDaList.ceDa1 = ceDa;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CE_DA_PA4_PREEMPTION_PRIORITY;// ���ȼ�����
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = CE_DA_PA4_SUB_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;//����DMAͨ�����ж�����
    }
    else if (ceDa->ceResource == PA5ADGI)
    {
        ceDa->ceExDaPar.ceExDMAChannel = DMA2_Channel4;
        ceDa->ceExDaPar.ceExTIMx = TIM7;
        ceDa->ceExDaPar.ceExDAC_Channel_x = DAC_Channel_2;
        ceDa->ceExDaPar.ceExGpiox = GPIOA;
        ceDa->ceExDaPar.ceExGpioPinx = GPIO_Pin_5;
        ceDaList.ceDa2 = ceDa;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CE_DA_PA5_PREEMPTION_PRIORITY;// ���ȼ�����
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = CE_DA_PA5_SUB_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
    }
    else
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    //�ж�����
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Gpio����
    GPIO_InitStructure.GPIO_Pin = ceDa->ceExDaPar.ceExGpioPinx; //PA4 DAC1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ceDa->ceExDaPar.ceExGpiox, &GPIO_InitStructure);
    
    //��ʱ������
    ceCalDaPrescalerAndPeriod(&ceDaTimBaseData, ceDa->convertIntervalNs);
    TIM_TimeBaseStructure.TIM_Prescaler = ceDaTimBaseData.timPrescaler;
    TIM_TimeBaseStructure.TIM_Period = ceDaTimBaseData.timPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//�����������ϼ���
    TIM_DeInit(ceDa->ceExDaPar.ceExTIMx);
    TIM_TimeBaseInit(ceDa->ceExDaPar.ceExTIMx, &TIM_TimeBaseStructure);
    TIM_PrescalerConfig(ceDa->ceExDaPar.ceExTIMx, ceDaTimBaseData.timPrescaler, TIM_PSCReloadMode_Update);
    TIM_SetAutoreload(ceDa->ceExDaPar.ceExTIMx, ceDaTimBaseData.timPeriod);
    TIM_SelectOutputTrigger(ceDa->ceExDaPar.ceExTIMx, TIM_TRGOSource_Update);

    //DMA����
    DMA_DeInit(ceDa->ceExDaPar.ceExDMAChannel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((ceDa->ceResource == PA4ADGI) ? CE_DAC1_Address : CE_DAC2_Address);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) &ceDaTemp;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(ceDa->ceExDaPar.ceExDMAChannel, &DMA_InitStructure);

    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼDAת��
  * @param   ceDa:Da���Զ���ָ��
  * @param   dataBuf:��Ҫת�������ݻ���
  * @param   dataBufSize:��Ҫת�������ݳ���
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceDa_start(CeDa* ceDa, const uint16* dataBuf, uint32 dataBufSize)
{
     DAC_InitTypeDef DAC_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__
    while (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00);
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x00;
    
    //DAC����
    if (ceDa->ceResource == PA4ADGI)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;//����TIMxΪ����Դ
        DAC_SetChannel1Data(DAC_Align_12b_R, 0);//����ADCָ��ͨ��������λ�������䷽ʽ���Լ��������ֵ�����ֵС��0xFFF��12λ�Ҷ���
    }
    else
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_T7_TRGO;
        DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    }
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//���ò����Ĳ��Σ����������ǲ����߲�����ǰ����������
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//��������Ƿ���Ҫ����
    DAC_Init(ceDa->ceExDaPar.ceExDAC_Channel_x, &DAC_InitStructure);

    //��������DMA������
    ceDa->ceExDaPar.ceExDMAChannel->CMAR = (uint32) dataBuf;//����DMAͨ�������Ļ�����
    ceDa->ceExDaPar.ceExDMAChannel->CNDTR = dataBufSize;//���û���������
    //ceDa->ceExDaPar.ceExTIMx->CNT = 0x0000;
    //ceDa->ceExDaPar.ceExTIMx->PSC = 0x0000;

    DMA_ITConfig(ceDa->ceExDaPar.ceExDMAChannel, DMA_IT_TC, ENABLE);//ʹ��DMAͨ���ж�    
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//ʹ��DACͨ��
    DAC_DMACmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//ʹ��DACͨ����DMA
    DMA_Cmd(ceDa->ceExDaPar.ceExDMAChannel, ENABLE);//ʹ��DMA
    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, ENABLE);//ʹ�ܶ�ʱ��
}

/**
  * @brief   ֹͣDaת��
  * @param   ceDa:Da���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceDa_stop(CeDa* ceDa)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, DISABLE);//ʧ�ܶ�ʱ��
    DAC_SoftwareTriggerCmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//ʧ���������
    DMA_Cmd(ceDa->ceExDaPar.ceExDMAChannel, DISABLE);//ʧ��DACͨ����DMA
    DAC_DMACmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//ʧ��DACͨ����DMA
    DMA_ITConfig(ceDa->ceExDaPar.ceExDMAChannel, DMA_IT_TC, DISABLE);//ʧ�ܷ���DMAͨ���ж�
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//ʧ��DACͨ��

    if (ceDa->ceResource == PA4ADGI)
    {
        DMA_ClearFlag(DMA2_FLAG_GL3);//����жϱ�־
    }
    else //if (ceDa->ceResource == R8ADIG)
    {
        DMA_ClearFlag(DMA2_FLAG_GL4);//����жϱ�־
    }
    
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x01;
}

/**
  * @brief   Da����̶�ֵ
  * @param   ceDa:Da���Զ���ָ��
  * @return  None
  */
void ceDa_startFixedVoltage(CeDa* ceDa, uint16 val)
{
    DAC_InitTypeDef DAC_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    while (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00);
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x00;

    if (val > 0x0FFF)
    {
        val = 0x0FFF;
    }

    //DAC����
    if (ceDa->ceResource == PA4ADGI)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;//�����޴���Դ
        DAC_SetChannel1Data(DAC_Align_12b_R, val);//����ADCָ��ͨ��������λ�������䷽ʽ���Լ��������ֵ�����ֵС��0xFFF��12λ�Ҷ���
    }
    else //if (ceDa->ceResource == R8ADIG)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
        DAC_SetChannel2Data(DAC_Align_12b_R, val);
    }
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//���ò����Ĳ��Σ����������ǲ����߲�����ǰ����������
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//��������Ƿ���Ҫ����
    DAC_Init(ceDa->ceExDaPar.ceExDAC_Channel_x, &DAC_InitStructure);

    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, ENABLE);
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//ʹ��DACͨ��
    DAC_SoftwareTriggerCmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//ʹ���������
}

/**
  * @brief   ����DA����
  * @param   ceDa:Da���Զ���ָ��
  */
void ceDa_update(CeDa* ceDa)
{
     CeDaTimBaseData ceDaTimBaseData;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__
    if (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00)//���Da�ڹ���״̬�򲻽����κβ���
    {
        return;
    }

    //��������TIM��ʱ��
    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, DISABLE);
    ceCalDaPrescalerAndPeriod(&ceDaTimBaseData, ceDa->convertIntervalNs);
    /* Set the Autoreload value */
    ceDa->ceExDaPar.ceExTIMx->ARR = ceDaTimBaseData.timPeriod;
    /* Set the Prescaler value */
    ceDa->ceExDaPar.ceExTIMx->PSC = ceDaTimBaseData.timPrescaler;

    TIM_SetCounter(ceDa->ceExDaPar.ceExTIMx, 0);
    TIM_ClearFlag(ceDa->ceExDaPar.ceExTIMx, TIM_FLAG_Update);//Clear TIMx update pending flag  ���TIMx����жϱ�־
    TIM_ITConfig(ceDa->ceExDaPar.ceExTIMx, TIM_IT_Update, ENABLE);
}

const CeDaOp ceDaOp = {ceDa_initial, ceDa_start, ceDa_startFixedVoltage, ceDa_stop, ceDa_update};

#ifdef __cplusplus
 }
#endif //__cplusplus
