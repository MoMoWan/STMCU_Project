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

#define CE_TMU_WIFI_SSID        "Darcern"           /*!< �������˻���Ҫ����WIFI��SSID*/
#define CE_TMU_WIFI_PWD         "Dxwzh178"          /*!< �������˻���Ҫ����WIFI������*/
#define CE_TMU_WIFI_SERVER_IP   "192.168.1.211"     /*!< �������˻���Ҫ���ӷ�������IP*/
#define CE_TMU_WIFI_SERVER_PORT  2121               /*!< �������˻���Ҫ���ӷ������Ķ˿�*/



/**
  * @brief  ö�٣���ǰ���˻�ͨѶ��ʽ
  */
typedef enum 
{
    CE_TMU_USE_WIFI,                     /*!< ʹ��Wifi���䷽ʽ*/
    CE_TMU_USE_BLUE,                     /*!< ʹ���������䷽ʽ*/
    CE_TMU_USE_NRF,                      /*!< ʹ��������Ƶ���䷽ʽ*/
}CE_TUM_USE;

/*
 *CeTMU���Զ���
 */
typedef struct
{
    CeWifiEsp   ceWifiEsp;              /*!< CeWifiEspģ����󣬻���ESP8266-12E*/
    CeBlueHc    ceBlueHc;               /*!< CeBlueHcģ����󣬻���HC-05*/
    CeWlsNrf    ceWlsNrf;               /*!< CeWlsNrfģ����󣬻���NRF24L01+*/
    uint32      sendPackCount;          /*!< ���͵����ݵĴ��������ڶ������*/
    uint32      recvPackCount;          /*!< ���յ����ݵĴ��������ڶ������*/
    void        (*recvCallBack)(uint8* recvBuf, uint16 recvCount);/*!< ���յ����ݺ�ֱ��TMU��������õĻص�*/
    uint32      lastSendTime;           /*!< ���ڼ������η���ʱ�����Ƿ����sendIntervalMs*/
    CE_TUM_USE  useType;                /*!< ��ǰ���˻�ͨѶ��ʽ*/
}CeTMU;
/*
 *CeTMU��������
 */
typedef struct
{
    CE_STATUS (*initial)(CE_TUM_USE useType, void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMUģ���ʼ��
                                                                     @param useType:����TMUģ��ʹ��ʲôͨѶ��ʽ�������ݴ���
                                                                     @param recvCallBack:�û����ṩ�Ļص�����*/

    CE_STATUS (*sendData)(uint8* dataBuf, uint16 dataCount);    /*!< @brief �������ݣ�ע�⣺�����ڲ����������һ�η������ݵ�ʱ���Ƿ����intervalMs�����С����ֱ�ӷ���
                                                                     @param dataBuf:���ͻ����ַ
                                                                     @param dataCount:���ͻ������ݳ���*/

    CE_STATUS (*checkConnectStatus)(void);                      /*!< @brief ����Ƿ�ͨѶ�ж�
                                                                     @return CE_STATUS_SUCCESS��ͨѶ������ ������ͨѶ�ж�*/

    uint32    (*getSendIntervalMs)(void);                       /*!< @brief ��������һ�η������ݵ���ʱ��ʱ����
                                                                     @return ������һ�η������ݵ����ڵ�ʱ��������λms*/
}CeTMUOp;
/*
 *CeTMU��������ʵ��
 */
extern const CeTMUOp ceTMUOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TMU_H__


