/**
  ******************************************************************************
  * @file    CeI2c.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeI2c���ļ�
  ******************************************************************************
  * @attention
  *
  *1)��Stm32f103�е�Ӳ��I2c Bug��Ϊ��֤������ȶ��ԣ�����������ģ�ⷽʽʵ�����е���ԴI2c
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeI2c.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#define CE_I2CMASTER_NOT_INITIAL  0x00      /*!< I2cMasterδʼ����־*/
#define CE_I2CMASTER_IS_IDLE      0x01      /*!< I2cMaster���б�־*/
#define CE_I2CMASTER_IS_BUSY      0x02      /*!< I2cMasteræµ��־*/

uint8   ceI2cMaster_status1 = 0x00;         /*!< I2cMaster����1״̬��ֵ����CE_I2CMASTER_NOT_INITIAL��ʾδʼ��
                                                 ֵ����CE_I2CMASTER_IS_IDLE��ʾ����
                                                 ֵ����CE_I2CMASTER_IS_BUSY��ʾæµ*/

uint8   ceI2cMaster_status2 = 0x00;         /*!< I2cMaster����2״̬��ֵ����CE_I2CMASTER_NOT_INITIAL��ʾδʼ��
                                                 ֵ����CE_I2CMASTER_IS_IDLE��ʾ����
                                                 ֵ����CE_I2CMASTER_IS_BUSY��ʾæµ*/

/**
  * @brief   ��ȡSDA���ŵĵ�ƽ
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  0x00:SDA����Ϊ�͵�ƽ��0x01:SDA����Ϊ�ߵ�ƽ
  */
uint8 ceGetI2cMasterSDABit(CeI2cMaster* ceI2cMaster)
{
    if ((ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->IDR & ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx) != (uint32)Bit_RESET)
    {
        return (uint8)Bit_SET;
    }
    else
    {
        return (uint8)Bit_RESET;
    }
}

/**
  * @brief   ֱ������SDA���ŵĹ���ģʽ
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @param   ceGpioMode:SDA���ŵ�����ģʽ
  * @return  None
  */
void ceSetI2cMasterSDAMode(CeI2cMaster* ceI2cMaster, uint8 isOut)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (isOut == 0x01)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else
    {
        //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//���ﲻӦ���ø������룬�����ʱ��Ҫ����һ��.
    }
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, &GPIO_InitStructure);
}

/**
  * @brief   ����豸���ͷ�Ӧ���ź�
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  None
  */
void ceI2cMasterNAck(CeI2cMaster* ceI2cMaster)
{
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);    //sda�����
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
}

/**
  * @brief   ����豸����Ӧ���ź�
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  None
  */
void ceI2cMasterAck(CeI2cMaster* ceI2cMaster)
{
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda�����
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   ��I2cMasterָ�������м���
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_NULL_POINTER��CE_STATUS_RESOURCE_ERROR
  */
CE_STATUS ceCheckCeI2cMaster(CeI2cMaster* ceI2cMaster)
{
    if (ceI2cMaster == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceI2cMaster->ceResource & CE_RES_MARK_I2C) != CE_RES_MARK_I2C))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ʼ��I2cMaster����
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_initial(CeI2cMaster* ceI2cMaster)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__

    switch ((uint32)(ceI2cMaster->ceResource))
    {
    case I2c1://I2c1,PB0,PB6,PB7
        ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx = &ceI2cMaster_status1;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx = GPIO_Pin_6;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx = GPIO_Pin_7;
        if(ceI2cMaster_status1 != CE_I2CMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceI2cMaster_status1 = CE_I2CMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        break;
    case I2c2://I2c2,PA8,PB10,PB11
        ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx = &ceI2cMaster_status2;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx = GPIO_Pin_10;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx = GPIO_Pin_11;
        if(ceI2cMaster_status2 != CE_I2CMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceI2cMaster_status2 = CE_I2CMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    switch (ceI2cMaster->ceI2cMasterSpeed)
    {
    case CE_I2C_SPEED_100KBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 10000;
        break;
    case CE_I2C_SPEED_400KBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 2500;
        break;
    case CE_I2C_SPEED_3_4MBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 250;
        break;
    default:
        break;
    }

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//���ﲻӦ���ø������룬�����ʱ��Ҫ����һ��.
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, &GPIO_InitStructure);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼI2cMaster�������
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceI2cMaster_start(CeI2cMaster* ceI2cMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda�����
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//������ʼ�����������ź�
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//������ʼ�ź�
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//ǯסI2C���ߣ�׼�����ͻ��������
}

/**
  * @brief   ֹͣI2cMaster�������
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceI2cMaster_stop(CeI2cMaster* ceI2cMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);//sda�����
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//���ͽ��������������ź�
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//������������ʱ�����4��s
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//����I2C���߽����ź�
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
}

/**
  * @brief   ����һ���ֽ�
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @param   val:Ҫ���͵��ֽ�
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceI2cMaster_sendByte(CeI2cMaster* ceI2cMaster, uint8 val)
{
    u8 BitCnt;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda�����
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//����ʱ�ӿ�ʼ���ݴ���
    //���� һ��Ҫ�������� ����SCL����0״̬ ���ܽ���д��
    for (BitCnt = 0; BitCnt < 8; BitCnt++)//Ҫ���͵����ݳ���Ϊ8λ
    {
        if ((val << BitCnt) & 0x80)//�жϷ���λ  �������ɸ�λ��ʼ����
        {
            ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
        }
        else
        {
            ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
        }
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    }
}

/**
  * @brief   ����һ���ֽڲ���
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @param   isAck:������ɺ��Ƿ���Ӧ����Ϣ��0x00:�����ͣ�0x01:����
  * @return  ��ȡ�����ֽ�
  */
uint8 ceI2cMaster_recvByte(CeI2cMaster* ceI2cMaster, uint8 isAck)
{
    u8 retc = 0, i;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x00);            //SDA����Ϊ����
    for (i = 0; i < 8; i++)
    {
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//��ʱ����Ϊ�ͣ�׼����������λ
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//��ʱ����Ϊ��ʹ��������������Ч
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
        retc <<= 1;
        if (GPIO_ReadInputDataBit(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx) > 0x00)
        {
            retc++;//������λ,���յ�����λ����retc��
        }
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    }
    if (isAck <= 0x00)
        ceI2cMasterNAck(ceI2cMaster);//����nACK
    else
        ceI2cMasterAck(ceI2cMaster);//����ACK
    return retc;
}

/**
  * @brief   �ȴ�I2cMaster���󷵻�Ӧ����Ϣ�����250��ʱ����������Ӧ���򷵻س�ʱ״̬�룬
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_OUT_TIME��CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_waitAck(CeI2cMaster* ceI2cMaster)
{
    u8 Time = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x00);//����SDAΪ����
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//׼������Ӧ��λ
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    while (GPIO_ReadInputDataBit(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx))
    {
        Time++;
        if (Time > 250)
        {
            ceI2cMaster_stop(ceI2cMaster);
            return CE_STATUS_OUT_TIME;//��Ӧ�𷵻س�ʱ
        }
    }
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//ʱ�����0
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ȡI2c���߿���Ȩ
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_OUT_TIME��CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_lockBus(CeI2cMaster* ceI2cMaster)
{
    uint16 temp = 0x0000;
    uint8* pI2cMaster_statusx = (uint8*)(ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx);
    while((*pI2cMaster_statusx) == CE_I2CMASTER_IS_BUSY)
    {
        ceSystemOp.delayNs (1);
        temp++;
        if(temp > 0x1770)//��ʱʱ��Ϊ6m
        {
            return CE_STATUS_OUT_TIME;
        }
    };
    *pI2cMaster_statusx = CE_I2CMASTER_IS_BUSY;//�����ȡ��ʹ��Ȩ�����̽�״̬λ��־Ϊæµ״̬��Ȼ�󷵻ػ�ȡ�ɹ�
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   �ͷ�I2c���߿���Ȩ
  * @param   ceI2cMaster:I2cMaster���Զ���ָ��
  * @return  None
  */
void ceI2cMaster_unlockBus(CeI2cMaster* ceI2cMaster)
{
    *(ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx) = CE_I2CMASTER_IS_IDLE;
}

const CeI2cMasterOp ceI2cMasterOp = {ceI2cMaster_initial, ceI2cMaster_start, ceI2cMaster_stop, ceI2cMaster_sendByte, ceI2cMaster_recvByte,
                                            ceI2cMaster_waitAck,ceI2cMaster_lockBus, ceI2cMaster_unlockBus};
#ifdef __cplusplus
 }
#endif //__cplusplus

