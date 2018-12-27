/**
  ******************************************************************************
  * @file    CeSpi.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeSpiͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_SPI_H__
#define __CE_SPI_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  ö�٣�SpiMaster���������
  */
typedef enum
{
    CE_SPI_MASTER_SPEED_1GBPS = 0x00,
    CE_SPI_MASTER_SPEED_500MBPS,
    CE_SPI_MASTER_SPEED_100MBPS,
    CE_SPI_MASTER_SPEED_50MBPS,
    CE_SPI_MASTER_SPEED_20MBPS,
    CE_SPI_MASTER_SPEED_10MBPS,
    CE_SPI_MASTER_SPEED_5MBPS,
    CE_SPI_MASTER_SPEED_1MBPS,
    CE_SPI_MASTER_SPEED_500KBPS,
    CE_SPI_MASTER_SPEED_100KBPS,
    CE_SPI_MASTER_SPEED_50KBPS,
    CE_SPI_MASTER_SPEED_10KBPS,
    CE_SPI_MASTER_SPEED_5KBPS,
    CE_SPI_MASTER_SPEED_1KBPS
}CE_SPI_MASTER_SPEED;

/**
  * @brief  ö�٣�SpiMaster����ʱ���ڿ���ʱ��״̬
  */
typedef enum
{
    CE_SPI_MASTER_CLOCK_POLARITY_HIGH = 0x00,           /*!< ����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ������豸��Ϊ�ߵ�ƽ������Ĭ��Ϊ��ֵ*/
    CE_SPI_MASTER_CLOCK_POLARITY_LOW                    /*!< ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ*/
}CE_SPI_MASTER_CLOCK_POLARITY;

/**
  * @brief  ö�٣�SpiMaster����ʱ���ڿ���ʱ��״̬
  */
typedef enum
{
    CE_SPI_MASTER_CLOCK_PHASE_2Edge = 0x00,             /*!< ����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����������豸��Ϊ�ڶ��������أ�����Ĭ��Ϊ��ֵ*/
    CE_SPI_MASTER_CLOCK_PHASE_1Edge                     /*!< ����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ���������������ģ�飬����: nRf24L01*/
}CE_SPI_MASTER_CLOCK_PHASE;

/**
  * @brief  �ṹ�壬SpiMaster����������Լ���
  */
typedef struct
{
    CE_RESOURCE         ceResource;                     /*!< SPI��Ӧ����Դ��*/
    CE_SPI_MASTER_SPEED ceSpiMasterSpeed;               /*!< SPI����������*/
    CE_SPI_MASTER_CLOCK_POLARITY ceSpiMasterClockPolarity;/*!< SPI����ʱͬ��ʱ�ӵĿ���״̬*/
    CE_SPI_MASTER_CLOCK_PHASE    ceSpiMasterClockPhase;/*!< SPI����ʱͬ��ʱ����ָ��������ʱ��������*/
    CeExSpiMasterPar    ceExSpiMasterPar;               /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeSpiMaster;

/**
  * @brief  �ṹ�壬SpiMaster������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeSpiMaster* ceSpiMaster);   /*!< @brief ��ʼ��SPI
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    void        (*start)(CeSpiMaster* ceSpiMaster);     /*!< @brief ��ʼSPI
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    void        (*stop)(CeSpiMaster* ceSpiMaster);      /*!< @brief ֹͣSPI
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    uint8       (*writeReadByte)(CeSpiMaster* ceSpiMaster, uint8 writeVal);/*!<
                                                             @brief ���Ͳ���������
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��
                                                             @param writeVal:Ҫ���͵����ݣ���λ�ֽ�
                                                             @return ���������ݣ���λ�ֽ�*/

    void        (*setNSSBit)(CeSpiMaster* ceSpiMaster); /*!< @brief ����SpiMaster��NSS���ŵ�ƽΪ��
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    void        (*resetNSSBit)(CeSpiMaster* ceSpiMaster);/*!<
                                                             @brief ����SpiMaster��NSS���ŵ�ƽΪ��
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    CE_STATUS   (*lockBus)(CeSpiMaster* ceSpiMaster);   /*!< @brief ��ȡSpi���߿���Ȩ
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

    void        (*unlockBus)(CeSpiMaster* ceSpiMaster); /*!< @brief �ͷ�Spi���߿���Ȩ
                                                             @param ceSpicMaster:SpiMaster���Զ���ָ��*/

}CeSpiMasterOp;
extern const CeSpiMasterOp ceSpiMasterOp;           /*!< ������SpiMaster��صĲ���*/
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_SPI_H__
