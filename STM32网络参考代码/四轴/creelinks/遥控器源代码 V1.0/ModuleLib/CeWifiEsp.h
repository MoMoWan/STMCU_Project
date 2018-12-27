/**
  ******************************************************************************
  * @file    CeWifiEsp.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeWifiEspģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_WIFI_ESP_H__
#define __CE_WIFI_ESP_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_WIFI_ESP_VERSION__ 1                                       /*!< �������ļ��İ汾��*/
#define __CE_WIFI_ESP_NEED_CREELINKS_VERSION__ 1                        /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_WIFI_ESP_NEED_CREELINKS_VERSION__) /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeWifiEsp.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else
#define CE_WIFI_ESP_UT                                                /*!< ESP8266�Ƿ�����͸��(Ĭ�Ϲر�)��ע�⣺������STA�ĵ�����ģʽ�ſ�������͸������������ã���ֻ����Ϊ�ͻ�������һ��������*/

#define CE_WIFI_ESP_RECV_BUF_SIZE       256                            /*!< UARTʹ�õ���fifo���������ɸ���ʵ��������ӻ���٣�����Ҫ��֤����һ�ε�������С�ڴ�ֵ*/
#define CE_WIFI_ESP_MAX_CONNECT_BUF     10                              /*!< �鿴���豸��ͻ��������б�����ֵ*/
/*
 *ö�� ��ģʽ�Ĺ�����ʽ
*/
typedef enum
{
    CE_WIFI_ESP_MODE_STA = 1,                                           /*!< STAģʽ: Station, �����������նˣ�sta�������������ߵĽ��룬���������ӵ�AP��һ�����������������ڸ�ģʽ*/
    CE_WIFI_ESP_MODE_AP = 2,                                            /*!< APģʽ: Access Point���ṩ���߽�������������������豸���룬�ṩ���ݷ��ʣ�һ�������·��/���Ź����ڸ�ģʽ��*/
    CE_WIFI_ESP_MODE_STA_AP=3,                                          /*!< AP+STA*/
}CE_WIFI_ESP_MODE;

/*
 *ö�� APģʽ�£�������Wifiʱ�ļ��ܷ�ʽ
*/
 typedef enum
{
    CE_WIFI_ESP_ECN_OPEN = 0,                                           /*!< ����ʽ*/
    CE_WIFI_ESP_ECN_WEP = 1,                                            /*!< WEP�����ߵ�Ч���ܣ���������WEP 64λ����128λ���ݼ���*/
    CE_WIFI_ESP_ECN_WPA_PSK = 2,                                        /*!< WPA-PSK[TKIP]��������Ԥ������Կ��Wi-Fi�������ʣ�����WPA-PSK��׼���ܼ�������������ΪTKIP*/
    CE_WIFI_ESP_ECN_WPA2_PSK = 3,                                       /*!< WPA2-PSK[AES]��������Ԥ������Կ��Wi-Fi�������ʣ��汾2��������WPA2-PSK��׼���ܼ�������������ΪAES*/
    CE_WIFI_ESP_ECN_WPA_WPA2_PSK = 4,                                   /*!< WPA-PSK[TKIP] + WPA2-PSK [AES]��������ͻ���ʹ��WPA-PSK [TKIP]����WPA2-PSK [AES]*/
}CE_WIFI_ESP_ECN;

/*
 *ö�� SOCKET�����ӷ�ʽ
*/
typedef enum
{
    CE_WIFI_ESP_SOCKET_MODE_TCP=0,                                      /*!< TCP���ӷ�ʽ*/
    CE_WIFI_ESP_SOCKET_MODE_UDP=1                                       /*!< UDP���ӷ�ʽ*/
}CE_WIFI_ESP_SOCKET_MODE;
/*
 *ö�� APģʽ�룬���ģ���������豸��Ϣ
*/
typedef struct
{
    char ip[16];                                                        /*!< IP��ַ���ַ���"192.168.1.1"��ʽ*/
    char mac[18];                                                       /*!< MAC��ַ���ַ���"8F:34:32:A4:56:76"��ʽ*/
}CeWifiEspAPLinkDevInfo;

/*
    *ö�� �Ѿ�������SOCKET������ϸ��Ϣ
*/
typedef struct
{
    CE_WIFI_ESP_SOCKET_MODE socketMode;                                 /*!< ��TCP����UDP*/
    char                    ip[16];                                     /*!< IP��ַ���ַ���"192.168.1.1"��ʽ*/
    uint16                  port;                                       /*!< �˿ں�*/
    uint8                   linkID;                                     /*!< �������ӵ�ID��ESP8266һ��������5������ͬʱ����*/
    uint8                   moduleLinkType;                             /*!< ָ�����ӣ�ģ���������Ƿ������Ľ�ɫ�����ǿͻ��˵Ľ�ɫ��1��Ϊ��������0��Ϊ�ͻ���*/
}CeWifiEspLinkInfo;

/*
 *ö�� �����п���Wifi����ϸ��Ϣ����
*/
typedef struct
{
    CE_WIFI_ESP_ECN ceWifiEpsEcn;                                       /*!< ���ܷ�ʽ*/
    char ssid[64];                                                      /*!< wifi��ssid����*/
    int signal;                                                         /*!< �ź�ǿ��*/
    char mac[18];                                                       /*!< Mac��ַ*/
    uint16 other1;                                                      /*!< */
    uint16 other2;                                                      /*!< */
}CeWifiEspCanConnectWifiInfo;

/*
 *CeWifiEsp���Զ���
 */
typedef struct
{
    CeUart              ceUart;                                                     /*!< ģ��ʹ�õ���UART��Դ�ӿڶ���*/
    CeTask              ceTask;                                                     /*!< ��ģ��ʹ�õ�������������Ҫ�𵽽������ݵļ�飬��ִ���û��ṩ�Ľ��ջص�����*/
    CE_WIFI_ESP_MODE    wifiEspMode;                                                /*!< ��ģ��Ĺ�����ʽ��AP��STA����*/
    CeWifiEspAPLinkDevInfo      apLinkDevInfoList[CE_WIFI_ESP_MAX_CONNECT_BUF];     /*!< APģʽ�£����ӵ���ģ���Wifi�µ��豸����*/
    CeWifiEspLinkInfo           linkInfoList[CE_WIFI_ESP_MAX_CONNECT_BUF];          /*!< ��������ͻ��������½�����SOCKET��������*/
    CeWifiEspCanConnectWifiInfo staCanConnectWifiList[CE_WIFI_ESP_MAX_CONNECT_BUF]; /*!< ģ�����Χ�Ŀ���Wifi�������������ҵ��Ŀ���Wifi����*/
    void    (*callBackClientRecv[5])(uint8* recvBuf, uint16 recvCount);             /*!< ���ڿͻ���ģʽʱ���ɽ���5���ͻ��˷ֱ��Ӧ�Ľ��ջص�*/
    void    (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount);   /*!< ���ڷ�����ģʽʱ���������յ����ݺ�ִ�еĻص�,linkID:�ڼ����ͻ��˷�����������*/
    uint8   isLockRecvBuf;                                                          /*!< �������ջ��棬��ֹ����ʱ�����ʱ���������ͻ*/
    uint8   isServerMode;                                                           /*!< ��ǰ�����Ǵ��ڷ����������ǿͻ���״̬�������жϵ��÷������ص������ǿͻ��˻ص�*/
    uint8   apLinkDeviceCount;                                                      /*!< APģʽ�£��ж��ٸ��豸���ӵ���ģ���Wifi��*/
    uint8   linkInfoCount;                                                          /*!< AP��STAģʽ�£��ж��ٸ�SOCKET���Ӵ��ڣ�UDP��TCP*/
    uint8   staCanConnectwifiCount;                                                 /*!< ��STAģʽ�£����ҵ��Ļ����еĿ���Wifi����*/
    uint8   isInUTSend;                                                             /*!< �ͷ��������£��Ƿ���͸��ģʽ��*/
    uint8   uartRecvBuf[CE_WIFI_ESP_RECV_BUF_SIZE];                                 /*!< Uart����ʹ�õ��Ľ��ջ���*/
    uint8   recvData[CE_WIFI_ESP_RECV_BUF_SIZE];                                    /*!< ��Ž��յ������ݣ������û�ֱ�ӵ��ô������ݴ���ص�������ɺ󣬴��е����ݲ��ܹ���ˢ�£��������ݾ������浽fifo��*/
}CeWifiEsp;

/*
 *CeWifiEsp��������
 */
typedef struct
{

    CE_STATUS   (*initial)(CeWifiEsp* ceWifiEsp,CE_RESOURCE ceUart);                                    /*!< @brief  ʹ��һ��UART��Դ����ʼ��CeWifiEspģ��
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  ceUart:��ģ��ʹ�õ���Uart��Դ�ӿ�*/

    CE_STATUS   (*setWorkMode)(CeWifiEsp* ceWifiEsp,CE_WIFI_ESP_MODE ceWifiEspMode);                    /*!< @brief  ��APģʽ��STAģʽ��ʼ��WIFI
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  ceWifiEspMode:��ģ��Ĺ�����ʽ����AP��STA��ʽ��
                                                                                                                    ��APģʽ��ģ����Ϊ�ȵ㣬�����豸����ģ�飩����ģ�����ã�
                                                                                                                    ��STA��ģ����Ϊ���豸�����������Ѵ��ڵ��ȵ㣬�����·�������ֻ��Ͻ������ȵ�ȣ�����ģ������*/


    CE_STATUS   (*createWifi)(CeWifiEsp* ceWifiEsp,const char* ip,const char* ssid, const char* passWord,uint8 channelNumber,CE_WIFI_ESP_ECN ceWifiEspEcn);
                                                                                                        /*!< @brief  ����һ��AP�ȵ�
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  ip:��AP�ȵ��IP��ַ���������ӵ���ģ����豸��IP�ڴ˻���������
                                                                                                             @param  ssid:��AP�ȵ��Wifi����
                                                                                                             @param  channelNumber:Wifi�����ŵ�����Χ1��13����ʼ��ʱ����Χ������ֵ���ɡ�
                                                                                                             @param  ceWifiEspEcn:��AP�ȵ㴴����Wifi���ܷ�ʽ������CE_WIFI_ESP_ECN_WPA2_PSK*/

    CeWifiEspAPLinkDevInfo* (*getConnectedDeviceList)(CeWifiEsp* ceWifiEsp);                            /*!< @brief  AP������ʽ�£�������AP�ݵ����ӵĴ��豸��IP��MAC��ַ����
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return ���豸��Ϣ����*/

    uint8       (*getConnectedDeviceCount)(CeWifiEsp* ceWifiEsp);                                       /*!< @brief  AP������ʽ�£�������AP�ݵ����ӵĴ��豸���������������ж������豸���ӵ���Wifi֮��
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return ���豸������*/

    CeWifiEspCanConnectWifiInfo* (*getCanConnectWifiList)(CeWifiEsp* ceWifiEsp);                        /*!< @brief  STA������ʽ�£�������Χ�����п����ӵ�Wifi�ź�
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return ����Wifi������ָ��*/

    uint8       (*getCanConnectWifiCount)(CeWifiEsp* ceWifiEsp);                                        /*!< @brief  STA������ʽ�£�������Χ�����п����ӵ�Wifi�ź�����
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return ����Wifi������*/

    CE_STATUS   (*checkCanConnectSsidIsExist)(CeWifiEsp* ceWifiEsp, const char* ssid);                  /*!< @brief  STA������ʽ�£�������Χ�������Ƿ��и�����ssid��Wifi�ź�
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���*/

    CE_STATUS   (*connectWifi)(CeWifiEsp* ceWifiEsp, const char* ssid, const char* passWord);           /*!< @brief  STA������ʽ������һ���Ѿ����ڵ��ȵ㣬����Ϊ�ȵ��SSID��PWD
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  ssid:��Ҫ���ӵ�Wifi����
                                                                                                             @param  passWord:Wifi����*/

    CE_STATUS   (*setStaIp)(CeWifiEsp* ceWifiEsp, const char* ip);                                      /*!< @brief  STA������ʽ�£����ӵ��ȵ���IP��ע���ʱ���õ�IPӦ���ȵ�����ID��ƥ�䡣
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  ip:��Ҫ�趨��IP��ַ*/

    CE_STATUS   (*createServer)(CeWifiEsp* ceWifiEsp,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount));
                                                                                                        /*!< @brief  ����һ�������������������뷢�����ݣ�AP��STAģʽ���ɴ���
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  serVerPortNum:�贴���ķ������˿ں�
                                                                                                             @param  linkID:�����Ǵ����˷���������˷��������ֻ����5���ͻ����������ӣ�linkIDָ�ľ�����5���ͻ��˵ı�ţ����������������Ŀͻ���Ϊ0���ص���LinkID��ʾ���������յ������ݣ����������ĸ��ͻ��˷�������
                                                                                                             @param  callBackServerRecv:���������յ����ݺ�Ļص�*/

    CE_STATUS   (*connectServer)(CeWifiEsp* ceWifiEsp, uint8 linkID, const char* serverIP,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackClientRecv)(uint8* recvBuf, uint16 recvCount));
                                                                                                        /*!< @brief  ����һ���Ѵ��ڵķ����������������뷢�����ݣ�AP��STAģʽ���ɴ���
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  linkID:ESP8266���Դ���5���ͻ��ˣ����0��4���ֱ����Ӹ��Եķ�������LinkIDָʹ���ĸ��ͻ��˱�ź����ӷ�����
                                                                                                             @param  serverIP:��Ҫ���ӵķ�������IP
                                                                                                             @param  serVerPortNum:��Ҫ���ӵķ������Ķ˿�
                                                                                                             @param ceWifiEspSocketMode:���ӷ�ʽ��TCP��UDP
                                                                                                             @param callBackClientRecv:�˿ͻ��˽��յ����ݺ�Ļص�*/

    CE_STATUS           (*startUTSendOnClient)(CeWifiEsp* ceWifiEsp);                                   /*!< @brief  ����͸����ֻ��ģ���ǵ����ӣ���Ϊ�ͻ��˲ſ����д˹���
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���*/

    CE_STATUS           (*stopUTSendOnClient)(CeWifiEsp* ceWifiEsp);                                    /*!< @brief  �˳�͸����ֻ��ģ���ǵ����ӣ���Ϊ�ͻ��˲ſ����д˹���
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���*/

    CeWifiEspLinkInfo*  (*getLinkList)(CeWifiEsp* ceWifiEsp);                                           /*!< @brief  ��ñ�ģ���������������б�����������ģʽ�£���˷��������ӵĿͻ����������ͻ���ģʽ�£��ж���ٿͻ������ڽ����������������ӣ�����5����
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return SOCKET��������*/

    uint8               (*getLinkCount)(CeWifiEsp* ceWifiEsp);                                          /*!< @brief  ��ñ�ģ�����������������������������жϣ�������������ʽ�£����޿ͻ������ӡ��ͻ��˷�ʽ�£��Ƿ��������ӵ���������
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @return SOCKET��������*/

    CE_STATUS           (*sendData)(CeWifiEsp* ceWifiEsp,uint8 linkID, uint8* dataBuf,uint16 dataSize); /*!< @brief  ��һ�����Ӻ�linkID�������ݡ�
                                                                                                             @param  ceWifiEsp:CeWifiEsp���Զ���
                                                                                                             @param  linkID:������ģʽ�£������Ӻ�ָ�����������ӵĿͻ�����ţ��ͻ���ģʽ�£������Ӻ�ָʹ���ĸ��ͻ��ˣ����0��4��
                                                                                                             @param  dataBuf:��Ҫ���͵����ݻ���
                                                                                                             @param  dataSize:��Ҫ���͵����ݳ���*/

}CeWifiEspOpBase;
/*
 *CeWifiEsp��������ʵ��
 */
extern const CeWifiEspOpBase ceWifiEspOp;

#endif //(__CE_CREELINKS_VERSION__ < __CE_WIFI_ESP_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_WIFI_ESP_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ʹ�������Ӽ�������·�ĵ��Ի��ֻ�����һ���������������ذ�װ����������֣���
          ����CeWifiEspģ�����Ӵ��ȵ㣨Wifi���������Ӹմ����ķ����������շ�����������
          �����ݣ���ͨ�����ڴ�ӡ��ͬʱÿ1S�����������һ�����ݰ���
******************************************************************************
#include "Creelinks.h"
#include "CeWifiEsp.h"
CeWifiEsp myWifiEsp;

void callBackClient0(uint8* recvBuf, uint16 recvCount)
{
    uint16 i;
    ceDebugOp.printf("CeWifiEsp has recv data from server, DataCount:%d, Data:\n",recvCount);
    for(i=0;i<recvCount;i++)
    {
        ceDebugOp.printf("%c",recvBuf[i]);
    }
    ceDebugOp.printf("\n");
}

int main(void)
{
    ceSystemOp.initial();                                        //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                                   //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceWifiEspOp.initial(&myWifiEsp,R14Uart,CE_WIFI_ESP_MODE_STA);//ʹ��R14Uart��Դ����ʹ��STA������ʽ����ģ����Ϊ�նˣ�����һ��Wifi��
    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.checkCanConnectSsidIsExist(&myWifiEsp,"Creelinks"))//ѭ����黷�����Ƿ����Creelinks��Wifi�źţ��û��������ѵ�Wifi�������޸�SSID��
    {
        ceDebugOp.printf("Cannot find wifi signal by ssid: Creelinks");
        ceSystemOp.delayMs(1000);
    }
    //ceWifiEspOp.setStaIp(&myWifiEsp,"192.168.1.188");//����ģ���IP��ַ����Ҫ��·��������һ�£�Ҳ�ɲ������ã�ģ�齫���Զ���ȡһ��IP��
    while(CE_STATUS_SUCCESS != ceWifiEspOp.connectServer(&myWifiEsp,0,"192.168.1.156",2121,CE_WIFI_ESP_SOCKET_MODE_TCP,callBackClient0))//����һ������������������úõķ����������ṩ������IP��˿ڡ�
    {
        ceDebugOp.printf("Cannot connect server,IP:192.168.1.156, Port:2121\n");
        ceSystemOp.delayMs(1000);
    }

    while (1)
    {
        ceTaskOp.mainTask();                           //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        ceWifiEspOp.sendData(&myWifiEsp,0,(uint8*)("Hello server!\n"),ceStringOp.strlen("Hello server!\n"));//����������������
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ʹ��CeWifiEsp����һ���������ȵ㣬������һ����������ͨ���ֻ���ʼǱ��������Ӵ�wifi�źţ����������ݰ���
******************************************************************************
#include "Creelinks.h"
#include "CeWifiEsp.h"
CeWifiEsp myWifiEsp;

void callBackServerRecv(uint8 linkID,uint8* recvBuf, uint16 recvCount)
{
    uint16 i;
    ceDebugOp.printf("CeWifiEsp has recv data from client, linkID:%d,  DataCount:%d, Data:\n",linkID,recvCount);
    for(i=0;i<recvCount;i++)
    {
        ceDebugOp.printf("%c",recvBuf[i]);
    }
    ceDebugOp.printf("\n");
}

int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceWifiEspOp.initial(&myWifiEsp,R14Uart,CE_WIFI_ESP_MODE_AP);//ʹ��R14Uart��Դ����ʹ��AP������ʽ����ģ����Ϊ·���������������豸����������
    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.createWifi(&myWifiEsp,"192.168.1.188","Creelinks","12345678",1,CE_WIFI_ESP_ECN_WPA2_PSK))//����һ��ָ��IP��Wifi�������롢���ܷ�ʽ��Wifi�ȵ㣬ʹ���ֻ���ʼǱ�������֮����
    {
        ceDebugOp.printf("Cannot create wifi!\n");
        ceSystemOp.delayMs(1000);
    }

    while(ceWifiEspOp.getConnectedDeviceCount(&myWifiEsp) == 0)                              //��鲢�ȴ������豸��֮������
    {
        ceDebugOp.printf("Wait for device connect wifi...\n");
        ceSystemOp.delayMs(1000);
    }

    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.createServer(&myWifiEsp,2121,callBackServerRecv))//����һ���˿�Ϊ2121�ķ�����
    {
        ceDebugOp.printf("Cannot create server!\n");
        ceSystemOp.delayMs(1000);
    }

    while(ceWifiEspOp.getLinkCount(&myWifiEsp) == 0)//��鲢�ȴ��ͻ�����֮����
    {
        ceDebugOp.printf("Wait for client connect server...\n");
        ceSystemOp.delayMs(1000);
    }
    while (1)
    {
        ceTaskOp.mainTask();                       //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        ceWifiEspOp.sendData(&myWifiEsp,0,(uint8*)("Hello server!\n"),ceStringOp.strlen("Hello server!\n"));//�����������ͻ���
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/


