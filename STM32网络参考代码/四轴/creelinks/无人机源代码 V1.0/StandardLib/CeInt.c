/**
  ******************************************************************************
  * @file    CeInt.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ����STM32F103RET6������ƽ̨��CeUart��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)��ͬ��Int��Դ���в�ͬ���ж����ȼ����û����Ժ궨��������,��ϸ���ù�����ο�Stm32f103������ɾ
  *2)ÿ��Int�Ļص������ж��ڽ��У���������ڻص���ִ�к�ʱ����
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeInt.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

GPIO_TypeDef*   ceIntGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
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

const uint8 ceIntPreemptionAndSubPriority[][16] = {     /*!< ��Դ�Ŷ�Ӧ���ж����ȼ�����PB5��Ӧ[1][5],0x31����λ��ʾ��ռʽ���ȼ�������λ����Ӧʽ���ȼ�*/
    {0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #ifdef GPIOE
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif

    #ifdef GPIOF
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif

    #ifdef GPIOG
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif
};

/**
  * @brief  �ṹ�壬�ⲿ�ж�Int�ݴ漯��
  */
CeInt* ceIntList[] = {CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL};


#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����ceIntָ�����
  * @param   ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeInt(CeInt* ceInt)
{
    if (ceInt == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceInt->callBack == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceInt->ceResource & CE_RES_MARK_INT) != CE_RES_MARK_INT))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   �ⲿ�ж�Int�ж���0����ں���
  * @param   None
  * @return  None
  */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line0);
        if(ceIntList[0]->ceExIntPar.ceExIsStart)
        {
            ceIntList[0]->callBack( ceIntList[0]->pAddPar);
        }
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���1����ں���
  * @param   None
  * @return  None
  */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line1);
        if(ceIntList[1]->ceExIntPar.ceExIsStart)
        {
            ceIntList[1]->callBack(ceIntList[1]->pAddPar);
        }
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���2����ں���
  * @param   None
  * @return  None
  */
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line2) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line2);
        if(ceIntList[2]->ceExIntPar.ceExIsStart)
        {
            ceIntList[2]->callBack(ceIntList[2]->pAddPar);
        }
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���3����ں���
  * @param   None
  * @return  None
  */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line3) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line3);
        if(ceIntList[3]->ceExIntPar.ceExIsStart)
        {
            ceIntList[3]->callBack( ceIntList[3]->pAddPar);
        }
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���4����ں���
  * @param   None
  * @return  None
  */
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line4);
        if(ceIntList[4]->ceExIntPar.ceExIsStart)
        {
            ceIntList[4]->callBack(ceIntList[4]->pAddPar);
        }
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���5��6��7��8��9����ں���
  * @param   None
  * @return  None
  */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line5) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line5);
        if(ceIntList[5]->ceExIntPar.ceExIsStart)
            ceIntList[5]->callBack( ceIntList[5]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line6) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line6);
        if(ceIntList[6]->ceExIntPar.ceExIsStart)
            ceIntList[6]->callBack(ceIntList[6]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line7);//��ʹû���õ����жϣ������Ǳ����ж��жϱ�־������жϱ�־�Ĵ��룬�Է�δ֪ԭ�����жϹ�������û������жϵ��µ��ж϶�ε���
        if(ceIntList[7]->ceExIntPar.ceExIsStart)
            ceIntList[7]->callBack(ceIntList[7]->pAddPar);    
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line8) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line8);
        if(ceIntList[8]->ceExIntPar.ceExIsStart)
            ceIntList[8]->callBack(ceIntList[8]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line9) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line9);       
        if(ceIntList[9]->ceExIntPar.ceExIsStart)
            ceIntList[9]->callBack(ceIntList[9]->pAddPar);
    }
}

/**
  * @brief   �ⲿ�ж�Int�ж���10��11��12��13��14��15����ں���
  * @param   None
  * @return  None
  */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line10) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line10);
        if(ceIntList[10]->ceExIntPar.ceExIsStart)
        {
            ceIntList[10]->callBack( ceIntList[10]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line11);
        if(ceIntList[11]->ceExIntPar.ceExIsStart)
        {
            ceIntList[11]->callBack(ceIntList[11]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line12);
        if(ceIntList[12]->ceExIntPar.ceExIsStart)
        {
            ceIntList[12]->callBack(ceIntList[12]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line13) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line13);//��Ȼû���õ����жϣ������Ǳ����ж��жϱ�־������жϱ�־�Ĵ��룬�Է�δ֪ԭ�����жϹ�������û������жϵ��µ��ж϶�ε���
        if(ceIntList[13]->ceExIntPar.ceExIsStart)
        {
            ceIntList[13]->callBack(ceIntList[13]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line14) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line14);//ͬ��
        if(ceIntList[14]->ceExIntPar.ceExIsStart)
        {
            ceIntList[14]->callBack(ceIntList[14]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line15);//ͬ��
        if(ceIntList[15]->ceExIntPar.ceExIsStart)
        {
            ceIntList[15]->callBack(ceIntList[15]->pAddPar);
        }
    }
}

/**
  * @brief   ��ʼ���ⲿ�ж�Int
  * @param   ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceInt_initial(CeInt* ceInt)
{
    NVIC_InitTypeDef* NVIC_InitStructure = &(ceInt->ceExIntPar.ceExNVIC_InitStructure);
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__


    GPIO_InitStructure.GPIO_Pin = (uint16)(0x0001) << (ceInt->ceResource & 0x0000000F);
    ceInt->ceExIntPar.ceExGpiox  = ceIntGpioxArray[(ceInt->ceResource>>4) & 0x0000000F];
    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line = 0x00000001 << ((ceInt->ceResource & 0x0000000F));
    ceIntList[ceInt->ceResource & 0x0000000F] = ceInt;
    NVIC_InitStructure->NVIC_IRQChannelPreemptionPriority = ceIntPreemptionAndSubPriority[(ceInt->ceResource>>4) & 0x0000000F][ceInt->ceResource & 0x0000000F] >>4;
    NVIC_InitStructure->NVIC_IRQChannelSubPriority = ceIntPreemptionAndSubPriority[(ceInt->ceResource>>4) & 0x0000000F][ceInt->ceResource & 0x0000000F] &0x0F;
    if((ceInt->ceResource) & (0x0000000F <= 4))
    {
        NVIC_InitStructure->NVIC_IRQChannel = (IRQn_Type)((ceInt->ceResource & 0x0000000F)+6);
    }else if(ceInt->ceResource & (0x0000000F <= 9))
    {
        NVIC_InitStructure->NVIC_IRQChannel = EXTI9_5_IRQn;
    }else
    {
        NVIC_InitStructure->NVIC_IRQChannel = EXTI15_10_IRQn;
    }

    switch (ceInt->ceIntMode)//�����жϴ�����ģʽ
    {
    case CE_INT_MODE_TRIGGER_FALLING:
        ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        break;
    case CE_INT_MODE_TRIGGER_RISING:
        ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        break;
    default:
        return CE_STATUS_INITIAL_FALSE;
    }

    ceInt->ceExIntPar.ceExGpioPinx = GPIO_InitStructure.GPIO_Pin;//��ȡ֮ǰ���õ�GpioPin
    ceInt->ceExIntPar.ceExIsStart = 0x00;//��ʼ��ʱ�����ⲿ�ж�IntΪ�ǹ���״̬

    RCC_APB2PeriphClockCmd(0x00000004 << ((ceInt->ceResource>>4) & 0x0000000F) , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    GPIO_Init(ceInt->ceExIntPar.ceExGpiox, &GPIO_InitStructure);
    GPIO_EXTILineConfig((ceInt->ceResource>>4) & 0x0000000F, ceInt->ceResource & 0x0000000F);

    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&(ceInt->ceExIntPar.ceExEXTI_InitStructure));
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));

    NVIC_InitStructure->NVIC_IRQChannelCmd = DISABLE;//��ʼ��ʱ��ر��ⲿ�ж�Int
    NVIC_Init(NVIC_InitStructure);

    EXTI_GenerateSWInterrupt(ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line);//����һ������жϣ���Ҳ�ܽ���Ϊʲô֮ǰ��һ��ʹ���ⲿ�ж�Int��������һ���жϵ������ˡ��˴������ɾ��������ʼ��ֹͣ�ⲿ�ж�Int����ǰ���������жϱ�־

    return CE_STATUS_SUCCESS;
}

/**
  * @brief  �����жϷ�ʽ
  * @param  ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @param  ceIntMode:�жϷ�ʽ
  * @return None
  */
void ceInt_setMode(CeInt* ceInt,  CE_INT_MODE ceIntMode)
{
    uint32 tmp = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__

    if (ceIntMode == CE_INT_MODE_TRIGGER_FALLING)
    {
        tmp = (uint32_t)EXTI_BASE;
        tmp += EXTI_Trigger_Falling;
        *(__IO uint32 *) tmp |= ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line;
    }
    else
    {
        tmp = (uint32_t)EXTI_BASE;
        tmp += EXTI_Trigger_Rising;

        *(__IO uint32 *) tmp |= ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line;
    }
}

/**
  * @brief   ��ʼ�ⲿ�ж�Int���
  * @param   ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @return  None
  */
void ceInt_start(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__
    ceInt->ceExIntPar.ceExIsStart = 0x01;//�����ⲿ�ж�IntΪ����״̬
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));//�ڿ��ж�ǰ���жϺ������һ���жϣ�Ԥ���ܽ��ϵ��źŶ���������Ȼ��ʼ�����һ�ο��жϻ���һ���жϡ�����EXTI_LineX��GPIO_Pin_X������XλΪ1��ʾĳ��Line��Pin����������ֱ���õ�GPIO_Pin_X��ʡ��1����Ա������
    ceInt->ceExIntPar.ceExNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&(ceInt->ceExIntPar.ceExNVIC_InitStructure));
}

/**
  * @brief   ֹͣ�ⲿ�ж�Int���
  * @param   ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @return  None
  */
void ceInt_stop(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__

    ceInt->ceExIntPar.ceExIsStart = 0x00;//�����ⲿ�ж�IntΪ������״̬

    if(ceInt->ceExIntPar.ceExGpioPinx > GPIO_Pin_4 && ceInt->ceExIntPar.ceExGpioPinx <= GPIO_Pin_9)//EXTI9_5_IRQn
    {
        if(ceIntList[5]->ceExIntPar.ceExIsStart || ceIntList[6]->ceExIntPar.ceExIsStart || ceIntList[7]->ceExIntPar.ceExIsStart || ceIntList[8]->ceExIntPar.ceExIsStart || ceIntList[9]->ceExIntPar.ceExIsStart)      
        {
            return;//���������1���ⲿ�ж�Int�ڿ�ʼ״̬�������κβ���
        }
    }
    else if(ceInt->ceExIntPar.ceExGpioPinx >= GPIO_Pin_10)//EXTI15_10_IRQn
    {
        if(ceIntList[10]->ceExIntPar.ceExIsStart || ceIntList[11]->ceExIntPar.ceExIsStart || ceIntList[12]->ceExIntPar.ceExIsStart
            || ceIntList[13]->ceExIntPar.ceExIsStart || ceIntList[14]->ceExIntPar.ceExIsStart || ceIntList[15]->ceExIntPar.ceExIsStart)
        {
            return;//���������1���ⲿ�ж�Int�ڿ�ʼ״̬�������κβ���������ִ�к������Ĵ���رմ��ж�
        }
    }

    ceInt->ceExIntPar.ceExNVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&(ceInt->ceExIntPar.ceExNVIC_InitStructure));
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));//�ڿ��ж�ǰ���жϺ������һ���жϣ�Ԥ���ܽ��ϵ��źŶ���������Ȼ��ʼ�����һ�ο��жϻ���һ���жϡ�����EXTI_LineX��GPIO_Pin_X������XλΪ1��ʾĳ��Line��Pin����������ֱ���õ�GPIO_Pin_X��ʡ��1����Ա������
}

/**
  * @brief   ��ȡ�ⲿ�ж�Int�ڶ�Ӧ��Gpio�ĵ�ƽֵ��0x01��0x00
  * @param   ceInt:�ⲿ�ж�Int���Զ���ָ��
  * @return  Gpio�ĵ�ƽֵ��0x01��0x00
  */
uint8 ceInt_getBit(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__
    return GPIO_ReadInputDataBit(ceInt->ceExIntPar.ceExGpiox, ceInt->ceExIntPar.ceExGpioPinx);
}

const CeIntOp ceIntOp = {ceInt_initial, ceInt_setMode, ceInt_start, ceInt_stop, ceInt_getBit};

#ifdef __cplusplus
 }
#endif //__cplusplus
