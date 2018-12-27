/**
  ******************************************************************************
  * @file    CeGpio.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeGpio��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeGpio.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus


GPIO_TypeDef*   ceGpioGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
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
void ceGpio_setMode(CeGpio* ceGpio, CE_GPIO_MODE ceGpioMode);

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����ceGpioָ�����
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeGpio(CeGpio* ceGpio)
{
    if (ceGpio == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceGpio->ceResource & CE_RES_MARK_GPIO) != CE_RES_MARK_GPIO) && ((ceGpio->ceResource & CE_RES_MARK_TG) != CE_RES_MARK_TG))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__


/**
  * @brief   ��ʼ��ceGpio
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceGpio_initial(CeGpio* ceGpio)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeGpio(ceGpio));
#endif //__CE_CHECK_PAR__

        //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(0x00000004 << ((ceGpio->ceResource>>4) & 0x0000000F) , ENABLE);
    ceGpio->ceExGpioPar.ceExGpiox = ceGpioGpioxArray[(ceGpio->ceResource>>4) & 0x0000000F];
    ceGpio->ceExGpioPar.ceExGpioPinx = (uint16)(0x0001) << (ceGpio->ceResource & 0x0000000F);

    ceGpio_setMode(ceGpio, ceGpio->ceGpioMode);

    return CE_STATUS_SUCCESS;
}
/**
  * @brief   ����ceGpio��ֵΪ1
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  None
  */
void ceGpio_setBit(CeGpio* ceGpio)
{
    ceGpio->ceExGpioPar.ceExGpiox->BSRR = ceGpio->ceExGpioPar.ceExGpioPinx;
}

/**
  * @brief   ����ceGpio��ֵΪ0
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  None
  */
void ceGpio_resetBit(CeGpio* ceGpio)
{
    ceGpio->ceExGpioPar.ceExGpiox->BRR = ceGpio->ceExGpioPar.ceExGpioPinx;
}

/**
  * @brief   ��ȡceGpio��ֵ
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  Gpio�ĵ�ƽֵ��0x01��0x00
  */
uint8 ceGpio_getBit(CeGpio* ceGpio)
{
    return GPIO_ReadInputDataBit(ceGpio->ceExGpioPar.ceExGpiox, ceGpio->ceExGpioPar.ceExGpioPinx);
}

/**
  * @brief   ����ceGpio�Ĺ���ģʽ
  * @param   ceGpio:CeGpio���Զ���ָ��
  * @return  None
  */
void ceGpio_setMode(CeGpio* ceGpio, CE_GPIO_MODE ceGpioMode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeGpio(ceGpio));
#endif //__CE_CHECK_PAR__

    ceGpio->ceGpioMode = ceGpioMode;
    GPIO_InitStructure.GPIO_Pin = ceGpio->ceExGpioPar.ceExGpioPinx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)(ceGpio->ceGpioMode);
    GPIO_Init(ceGpio->ceExGpioPar.ceExGpiox, &GPIO_InitStructure);
}

const CeGpioOp ceGpioOp = {ceGpio_initial, ceGpio_setBit, ceGpio_resetBit, ceGpio_getBit, ceGpio_setMode};

#ifdef __cplusplus
 }
#endif //__cplusplus
