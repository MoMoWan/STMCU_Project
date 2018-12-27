/**
  ******************************************************************************
  * @file    CeSpi.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeSpi���ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeSpi.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#define CE_SPIMASTER_NOT_INITIAL  0x00          /*!< SpiMasterδʼ����־*/
#define CE_SPIMASTER_IS_IDLE      0x01          /*!< SpiMaster���б�־*/
#define CE_SPIMASTER_IS_BUSY      0x02          /*!< SpiMasteræµ��־*/

uint8   ceSpiMaster_status1 = 0x00;             /*!< SpiMaster����1״̬��ֵ����CE_SPIMASTER_NOT_INITIAL��ʾδʼ��
                                                                         ֵ����CE_SPIMASTER_IS_IDLE��ʾ����
                                                                         ֵ����CE_SPIMASTER_IS_BUSY��ʾæµ*/

uint8   ceSpiMaster_status2 = 0x00;             /*!< SpiMaster����2״̬��ֵ����CE_SPIMASTER_NOT_INITIAL��ʾδʼ��
                                                                         ֵ����CE_SPIMASTER_IS_IDLE��ʾ����
                                                                         ֵ����CE_SPIMASTER_IS_BUSY��ʾæµ*/

uint8   ceSpiMaster_status3 = 0x00;             /*!< SpiMaster����3״̬��ֵ����CE_SPIMASTER_NOT_INITIAL��ʾδʼ��
                                                                         ֵ����CE_SPIMASTER_IS_IDLE��ʾ����
                                                                         ֵ����CE_SPIMASTER_IS_BUSY��ʾæµ*/

#ifdef __CE_CHECK_PAR__
/**
  * @brief   SpiMasterָ�������ȷ�Լ�飬�ڵ��Դ�__CE_CHECK_PAR__ʹ��
  * @param   ceSpicMaster:SpiMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_NULL_POINTER��CE_STATUS_RESOURCE_ERROR��CE_STATUS_SUCCESS
  */
CE_STATUS ceCheckSpiMaster(CeSpiMaster* ceSpiMaster)
{
    if (ceSpiMaster == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceSpiMaster->ceResource & CE_RES_MARK_SPI) != CE_RES_MARK_SPI))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   SpiMaster���󣬷��Ͳ�����һ�ֽ�����
  * @param   ceSpicMaster:SpiMaster���Զ���ָ��
  * @param   writeVal:Ҫ���͵����ݣ���λ�ֽ�
  * @return  ���������ݣ���λ�ֽ�
  */
uint8 ceSpiMasterByte_writeRead(CeSpiMaster* ceSpiMaster, uint8 writeVal)
{
    uint8 retry = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    while (SPI_I2S_GetFlagStatus(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
    {
        retry++;
        if (retry > 200) //��ʱ�˳�
            return 0;
    }
    SPI_I2S_SendData(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, writeVal); //ͨ������SPIx����һ������
    retry = 0;

    while (SPI_I2S_GetFlagStatus(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
    {
        retry++;
        if (retry > 200) //��ʱ�˳�
            return 0;
    }
    return SPI_I2S_ReceiveData(ceSpiMaster->ceExSpiMasterPar.ceExSPIx); //����ͨ��SPIx������յ�����

}

/**
  * @brief   ��ʼ��SpiMaster������Ҫ��ȫִ�У������ڱ�����ִ�������� ceSpiMaster_start
  * @param   ceSpicMaster:SpiMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceSpiMaster_initial(CeSpiMaster* ceSpiMaster)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    uint16 SPI_BaudRatePrescaler;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__



    switch ((uint32)(ceSpiMaster->ceResource))//SPI1ʱ�ӽ���APB2�ϣ���72MHz��SPI2��SPI3ʱ�ӽ���APB1�ϣ���36MHz��
    {
    case Spi1://SPI1
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status1;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI1;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_4;//PA4 SPI2 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_5;//PA5 SPI2 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_7;//PA7 SPI2 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_6;//PA6 SPI2 MISO
        if(ceSpiMaster_status1 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status1 = CE_SPIMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        break;
    case Spi2://SPI2
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status2;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI2;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_12;//PB12 SPI2 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_13;//PB13 SPI2 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_15;//PB15 SPI2 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_14;//PB14 SPI2 MISO
        if(ceSpiMaster_status2 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status2 = CE_SPIMASTER_IS_IDLE;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);       
        break;
    case Spi3://SPI3
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status3;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI3;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_15;//PA15 SPI3 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_3;//PB3 SPI3 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_5;//PB5 SPI3 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_4;//PB4 SPI3 MISO
        if(ceSpiMaster_status3 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status3 = CE_SPIMASTER_IS_IDLE;
        //Ϊ�����ô��е��Խӿ����ͷ�һЩ��ͨI/O�ڣ��û���������ڸ�λ������SWJ_CFG=010���Ӷ��ͷ�PA15��PB3��PB4������ͨI/O�ڡ�(������STM32���Ĳο��ֲ� 29.4.4)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);        
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    ceSpiMaster->ceExSpiMasterPar.ceExSPIx->CR1 &= 0XFFC7; //����SPI�ٶ�
    if (ceSpiMaster->ceSpiMasterSpeed <= CE_SPI_MASTER_SPEED_50MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_20MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_10MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_5MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_1MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_500KBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    else if (ceSpiMaster->ceSpiMasterSpeed >= CE_SPI_MASTER_SPEED_100KBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox, &GPIO_InitStructure);

    GPIO_SetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);//Ƭѡ���ߣ��͵�ƽ��Ч����ȷ�������豸�����������
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox, ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx);
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox, ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx);
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox, ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       //����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = (ceSpiMaster->ceSpiMasterClockPolarity == CE_SPI_MASTER_CLOCK_POLARITY_LOW) ? SPI_CPOL_Low : SPI_CPOL_High;//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = (ceSpiMaster->ceSpiMasterClockPhase == CE_SPI_MASTER_CLOCK_PHASE_1Edge) ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                            //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler;     //����ָ��������Ԥ��Ƶֵ���������Ϊ2��Ƶ��SPI2��SPI3����APB1�����裬ʱ��Ƶ�����Ϊ36M��SPI1����APB2�����裬ʱ��Ƶ�����Ϊ36M
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                   //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;                             //CRCֵ����Ķ���ʽ
    SPI_Init(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, &SPI_InitStructure);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼSpiMaster������ceSpiMaster_initial֮�����
  * @param   ceSpicMaster:SpiMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS
  */
void ceSpiMaster_start(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    if (ceSpiMaster->ceExSpiMasterPar.ceExSPIx != CE_NULL)
    {
        SPI_Cmd(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, ENABLE);//ʹ��SPI
    }
}

/**
  * @brief   ֹͣSpiMaster
  * @param   ceSpicMaster:SpiMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS
  */
void ceSpiMaster_stop(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    if (ceSpiMaster->ceExSpiMasterPar.ceExSPIx != CE_NULL)
    {
        SPI_Cmd(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, DISABLE);
    }
}

/**
  * @brief    ����SpiMaster��NSS���ŵ�ƽΪ��
  * @param    ceSpicMaster:SpiMaster���Զ���ָ��
  * @return   None
  */
void ceSpiMaster_setNSSBit(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    GPIO_SetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);    
}

/**
  * @brief    ����SpiMaster��NSS���ŵ�ƽΪ��
  * @param    ceSpicMaster:SpiMaster���Զ���ָ��
  * @return   None
  */
void ceSpiMaster_resetNSSBit(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);
}

/**
  * @brief   ��ȡSpi���߿���Ȩ
  * @param   ceSpiMaster:SpiMaster���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_OUT_TIME��CE_STATUS_SUCCESS
  */
CE_STATUS ceSpiMaster_lockBus(CeSpiMaster* ceSpiMaster)
{
    uint16 temp = 0x0000;
    uint8* pSpiMaster_statusx = (uint8*)(ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx);

    while((*pSpiMaster_statusx) == CE_SPIMASTER_IS_BUSY)
    {
        ceSystemOp.delayMs (1);
        temp++;
        if(temp > 0x1770)//��ʱʱ��Ϊ6m
        {
            return CE_STATUS_OUT_TIME;
        }
    };
    *pSpiMaster_statusx = CE_SPIMASTER_IS_BUSY;//�����ȡ��ʹ��Ȩ�����̽�״̬λ��־Ϊæµ״̬��Ȼ�󷵻ػ�ȡ�ɹ�
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   �ͷ�Spi���߿���Ȩ
  * @param   ceSpiMaster:SpiMaster���Զ���ָ��
  * @return  None
  */
void ceSpiMaster_unlockBus(CeSpiMaster* ceSpiMaster)
{
    *(ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx) = CE_SPIMASTER_IS_IDLE;
}

const CeSpiMasterOp ceSpiMasterOp = {ceSpiMaster_initial, ceSpiMaster_start, ceSpiMaster_stop, ceSpiMasterByte_writeRead, 
                                            ceSpiMaster_setNSSBit, ceSpiMaster_resetNSSBit, ceSpiMaster_lockBus, ceSpiMaster_unlockBus};

#ifdef __cplusplus
 }
#endif //__cplusplus
