/**
  ******************************************************************************
  * @file   CeAd.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinksƽ̨��Ad�����ʵ�ֺ���������STM32F103xƽ̨
  ******************************************************************************
  * @attention
  *
  *1)����Ad���õ�ͬһ��Adת��ģ�飬��ȡѭ���ɼ���ʽת��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeAd.h"
#include "CeSystem.h"

#ifdef __cplusplus
 extern "C" {
#endif  //__cplusplus

#define CE_AD_ADC1_DR_Address   ((uint32)0x4001244C)
#define CE_AD_NOT_INITIAL       0x00    /*!< ADδʼ����־*/
#define CE_AD_IS_IDLE           0x01    /*!< AD���б�־*/
#define CE_AD_IS_BUSY           0x02    /*!< ADæ��־*/

uint8   ceAd_status = 0x00;             /*!< AD��״̬��ֵ����CE_AD_NOT_INITIAL��ʾδʼ��*/
GPIO_TypeDef*   ceAdGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
#ifdef GPIOE
,GPIOE
#endif

#ifdef GPIOF
,GPIOF
#endif

#ifdef GPIOG
,GPIOG
#endif
}; 

const uint8 ceAdChannelxArray[][16] =
{
    {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x08,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

#ifdef GPIOE
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif

#ifdef GPIOF
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif

#ifdef GPIOG
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif
};



#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����Ad���Զ����ֵ�Ƿ���ȷ
  * @param   ceAd:Ad���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeAd(CeAd* ceAd)
{
    if (ceAd == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if ((ceAd->ceResource & CE_RES_MARK_AD) != CE_RES_MARK_AD)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   Ad�ɼ���ɺ�DMA������ɺ�Ļص�
  * @param   None
  * @return  None
  */
void DMA1_Channel1_IRQHandler()
{
}

/**
  * @brief   ��ʼ��Adת��
  * @param   ceAd:Ad���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceAd_initial(CeAd* ceAd)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeAd(ceAd));
#endif //__CE_CHECK_PAR__

    RCC_APB2PeriphClockCmd(0x00000004 << ((ceAd->ceResource>>4) & 0x0000000F) , ENABLE);
    ceAd->ceExPar.ceExGpiox = ceAdGpioxArray[(ceAd->ceResource>>4) & 0x0000000F];
    ceAd->ceExPar.ceExGpioPinx = (uint16)(0x0001) << (ceAd->ceResource & 0x0000000F);
    ceAd->ceExPar.ceAdChannelx =ceAdChannelxArray[(ceAd->ceResource & 0x000000F0) >> 4][ceAd->ceResource & 0x0000000F];

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//��ȡ��Ӧ��Adͨ����Goio�ڣ�����ʼ��Gpio
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = ceAd->ceExPar.ceExGpioPinx;
    GPIO_Init(ceAd->ceExPar.ceExGpiox, &GPIO_InitStructure);

    if(ceAd_status == CE_AD_NOT_INITIAL)
    {
        ADC_InitTypeDef ADC_InitStructure;
        ceAd_status = CE_AD_IS_IDLE;

        RCC_ADCCLKConfig(RCC_PCLK2_Div4);//����ADC����ʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ʹ��ADCʱ��

        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = DISABLE;//����ת��ģʽ
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת������������
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�����Ҷ���
        ADC_InitStructure.ADC_NbrOfChannel = 1;//ɨ��ͨ����
        ADC_Init(ADC1, &ADC_InitStructure);//��ʼ��ADC
        ADC_Cmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC

        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼAdת�����������Adת�����
  * @param   None
  * @return  ����Adת�����
  */
uint32 ceAd_getConvertValue(CeAd* ceAd)
{
    uint32 adConvertValue = 0x00000000;
    while(ceAd_status == CE_AD_IS_BUSY)
    {
        ceSystemOp.delayMs(0);
    }
    ceAd_status = CE_AD_IS_BUSY;

    ADC_RegularChannelConfig(ADC1, ceAd->ceExPar.ceAdChannelx, 1, ADC_SampleTime_1Cycles5);//����ת��ͨ����
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);//�����־
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);//�ȴ�ת������
    adConvertValue = ADC_GetConversionValue(ADC1);//��ȡADCת��ֵ
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
    ceAd_status = CE_AD_IS_IDLE;
    return adConvertValue;
}

const CeAdOp ceAdOp = {ceAd_initial, ceAd_getConvertValue};

#ifdef __cplusplus
 }
#endif  //__cplusplus
