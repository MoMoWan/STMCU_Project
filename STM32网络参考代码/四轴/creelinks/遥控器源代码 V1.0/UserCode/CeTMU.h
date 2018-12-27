/**
  ******************************************************************************
  * @file    CeTMU.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ���ݴ����������WIFI��������2.4Gģ��ĳ�ʼ�������ݷ��ͽ��յȴ���
  ******************************************************************************
  * @attention
  *
  *1)��ֲ��ע�⣺����initial�����У��������ģ��ʹ�õ�����Դ��
  *2)�������ݵ���send����������Byte���鼴�ɣ�
  *3)���յ����ݺ��Զ����ó�ʼ��ʱ�ṩ�Ļص�������δ���κδ����Byte���顣
  *4)���յ����ݺ󣬵��õĻص���������ceTaskOp.mainTask()��ִ�У��뱣֤��main�����е�ceTaskOp.mainTask()�ܹ������ڵ��� 
  * 
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TMU_H__
#define __CE_TMU_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"

#include "CeWifiEsp.h"
#include "CeBlueHc.h"
#include "CeWlsNrf.h"

#define CE_TMU_WIFI_SSID        "Darcern"              /*!< �������˻���Ҫ����WIFI��SSID*/
#define CE_TMU_WIFI_PWD         "Dxwzh178"          /*!< �������˻���Ҫ����WIFI������*/
#define CE_TMU_WIFI_SERVER_IP   "192.168.1.244"     /*!< �������˻���Ҫ���ӷ�������IP*/
#define CE_TMU_WIFI_SERVER_PORT  2121               /*!< �������˻���Ҫ���ӷ������Ķ˿�*/

#define CE_TMU_NRF_FIFO_SIZE    256

/**
  * @brief  ö�٣���ǰ���˻�ͨѶ��ʽ
  */
typedef enum 
{
    CE_TMU_USE_WIFI,                     /*!< ʹ��Wifi���䷽ʽ*/
    CE_TMU_USE_BLUE,                     /*!< ʹ���������䷽ʽ*/
    CE_TMU_USE_NRF,                      /*!< ʹ��������Ƶ���䷽ʽ*/
}CE_TUM_USE;

/**
  * @brief  ö�٣����˻���ǰ���ܿط�ʽ
  */
typedef enum 
{
    CE_TMU_CTL_TYPE_NONE=0x00,          /*!< δ�ܿ���*/
    CE_TMU_CTL_TYPE_STATION=0x01,       /*!< �ܵ���վ�����*/
    CE_TMU_CTL_TYPE_CONTROL=0x02,       /*!< ҡ��������*/
    CE_TMU_CTL_TYPE_PHONE=0x03,         /*!< ���ֻ�����*/
}CE_TUM_CTL_TYPE;
/*
 *CeTMU���Զ���
 */
typedef struct
{
    CeWifiEsp   ceWifiEsp;              /*!< CeWifiEspģ����󣬻���ESP8266-12E*/
    CeBlueHc    ceBlueHc;               /*!< CeBlueHcģ����󣬻���HC-05*/
    CeWlsNrf    ceWlsNrf;               /*!< CeWlsNrfģ����󣬻���NRF24L01+*/
    CeTask      recvTask;               /*!< ����nrf24l01ʹ���жϽ������ݣ��ص�Ҳ�����ж���ִ�У������ｨ������ֻ��nrf�����ж�д���ݵ�fifo��Ȼ�������������д���*/
    CeFifo      ceFifo;                 /*!< �������ݻ���FiFO*/

    uint8       stackBuf[CE_TMU_NRF_FIFO_SIZE];
    uint8       fifoBuf[CE_TMU_NRF_FIFO_SIZE];
    uint8       useBuf[CE_TMU_NRF_FIFO_SIZE];
    uint32      sendPackCount;          /*!< ���͵����ݵĴ��������ڶ������*/
    uint32      recvPackCount;          /*!< ���յ����ݵĴ��������ڶ������*/
    void        (*recvCallBack)(uint8* recvBuf, uint16 recvCount);/*!< ���յ����ݺ�ֱ��TMU��������õĻص�*/
    void        (*sendCallBack)(void);
    CE_TUM_USE  useType;                /*!< ��ǰ���˻�ͨѶ��ʽ*/
}CeTMU;
/*
 *CeTMU��������
 */
typedef struct
{
    CE_STATUS (*initialByWifi)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMUģ���ʼ��
                                                                     @param intervalMs:���巢��ʱ����
                                                                     @param recvCallBack:�û����ṩ�Ļص�����*/

    CE_STATUS (*initialByNrf)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount), void (*sendCallBack)(void)); /*!< 
                                                                     @brief CeTMUģ���ʼ��
                                                                     @param intervalMs:���巢��ʱ����
                                                                     @param recvCallBack:�û����ṩ�Ļص�����
                                                                     @param sendCallBack:����NRF24L01�����ݽ��������ж��У���Ϊ�ﵽ�յ����ݺ����̻ش����ݵ�һӦһ���ܣ��ж��յ����ݲ�д����FIFO�󣬻�ֱ�ӵ���sendCallBack���û�������д������ز���*/
 
    CE_STATUS (*initialByBlue)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMUģ���ʼ��
                                                                     @param intervalMs:���巢��ʱ����
                                                                     @param recvCallBack:�û����ṩ�Ļص�����*/

    CE_STATUS (*sendData)(uint8* dataBuf, uint16 dataCount);    /*!< @brief �������ݣ�ע�⣺�����ڲ����������һ�η������ݵ�ʱ���Ƿ����intervalMs�����С����ֱ�ӷ���
                                                                     @param dataBuf:���ͻ����ַ
                                                                     @param dataCount:���ͻ������ݳ���*/

    CE_STATUS (*getConnectStatus)(void);                        /*!< @brief ����Ƿ�ͨѶ�ж�
                                                                     @return CE_STATUS_SUCCESS��ͨѶ������ ������ͨѶ�ж�*/

}CeTMUOp;
/*
 *CeTMU��������ʵ��
 */
extern const CeTMUOp ceTMUOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TMU_H__


