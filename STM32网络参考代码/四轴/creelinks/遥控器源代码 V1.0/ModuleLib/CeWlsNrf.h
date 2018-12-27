/**
  ******************************************************************************
  * @file    CeWlsNrf.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeWlsNrfģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_WLS_NRF_H__
#define __CE_WLS_NRF_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_WLS_NRF_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_WLS_NRF_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_WLS_NRF_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeWlsNrf.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

#define CE_WLS_NRF_RECV_BUF_SIZE    128             /*!< ���ջ�������鳤��*/

#define CE_WLS_NRF_PACKET_LENGTH    32              /*!< ���ý����뷢�͵���Pack���ĳ���*/

typedef struct
{
    CeSpiMaster ceSpiMaster;                        /*!< ģ��ʹ�õ�Spi��Դ*/
    CeGpio      ceGpio;                             /*!< ģ��ʹ�õ�Gpio��Դ*/
    CeInt       ceInt;                              /*!< ģ��ʹ�õ�Int��Դ*/
    void        (*callBackRecv[6])(uint8* dataBuf, uint16 dataBufSize);/*!< ����ͨ��*/
    uint8       recvAddress[6][5];                  /*!< ���յ�ַ����*/
    uint8       recvBuf[CE_WLS_NRF_RECV_BUF_SIZE];  /*!< ���ջ���*/
    uint16      recvBufSize;                        /*!< ���ջ���������Ч���ݵ��ֽ���*/
    uint8       status;                             /*!< ����״̬���Ǵ��ڷ���״̬���ǽ���״̬*/
}CeWlsNrf;
/*
*CeWlsNrf��������
*/
typedef struct
{
    CE_STATUS   (*initial)(CeWlsNrf* ceWlsNrf, CE_RESOURCE ceSpi, CE_RESOURCE ceGpio,CE_RESOURCE ceInt);/*!<
                                                         @brief CeWlsNrfģ���ʼ��
                                                         @param ceWlsNrf:CeWlsNrf���Զ���ָ��
                                                         @param ceSpi:ģ��ʹ�õ�Spi��Դ��
                                                         @param ceGpio:ģ��ʹ�õ�ceGpio��Դ��
                                                         @param ceInt:ģ��ʹ�õ�ceInt��Դ��
                                                         @param ��ģ�����Ч��ַ����5���ֽڣ�һ��ģ���Ӧһ����ַ*/

    CE_STATUS   (*send)(CeWlsNrf* ceWlsNrf, uint8* sendAddress, uint8* dataBuf, uint16 dataBufSize);/*!<
                                                         @brief ���뷢��ģʽ�����Ͳ�����ɺ󣬺����ŷ���
                                                         @param ceWlsNrf:CeWlsNrf���Զ���ָ��
                                                         @param sendAddress:���͵�ַ������ն�6������ͨ���е�һ����ͬ
                                                         @param dataBuf:Ҫ���͵����ݻ�����
                                                         @param dataBufSize:Ҫ���͵����ݳ��ȣ�ע�⣺һ��ҪΪCE_WLS_NRF_PACKET_LENGTH��������
                                                         @return ����CE_STATUS_SUCCESS��������ͳɹ����������������ʧ��*/

    CE_STATUS   (*recv)(CeWlsNrf* ceWlsNrf, uint8 pipeIndex, uint8* recvAddress, void(callBackRecv)(uint8* dataBuf, uint16 dataBufSize));/*!<
                                                         @brief �������״̬����ʼ�������ݡ�ע�⣺������send������һ��Ҫ�ٴε��ô˺�������ʵ�����ݽ��գ��첽ִ�У�����ֱ�ӷ��ء�
                                                         @param ceWlsNrf:CeWlsNrf���Զ���ָ��
                                                         @param pipeIndex:ģ�鹲��6�����ý���ͨ������ֵָ��ʹ���ĸ�����ͨ���������ݣ����Ҫʹ�ö������ͨ���������ݣ�����ظ����ô˺������
                                                         @param recvAddress:����ͨ����Ӧ�Ľ��յ�ַ�����յ�ַ����ϸ������ϸ�Ķ�ģ���ֲ�
                                                         @param callBackRecv:����Ӧͨ�����յ����ݺ󣬵��õĻص�������ÿ��ͨ���Ļص�����������
                                                         @return ����CE_STATUS_SUCCESS��������óɹ����������������ʧ��*/
}CeWlsNrfOpBase;
/*
*CeWlsNrf��������ʵ��
*/
extern const CeWlsNrfOpBase ceWlsNrfOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_WLS_NRF_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_WLS_NRF_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ʹ��CeWlsNrfģ�齨�����ͻ���նˣ�ͨ���궨����ѡ��
          ���ͷ�1S����һ�����ݣ��������͵����ݼ����͵�״̬ͨ�����ڴ�ӡ
          ���շ������յ������ݺͽ��յĴ���ͨ�����ڴ�ӡ
******************************************************************************
#include "Creelinks.h"
#include "CeWlsNrf.h"

CeWlsNrf myWlsNrf;

CE_STATUS ceStatus = CE_STATUS_SUCCESS;               //��������̬
uint8 datBuffer[CE_WLS_NRF_PACKET_LENGTH] = { 0 };    //���ݻ�����
#define WLS_NRF_SEND                                  //ѡ���Ƿ��ͻ��ǽ��գ�����ǽ��������δ˺꣡

#ifdef WLS_NRF_SEND
uint8 sendAddress[5] = {0x34, 0xC3, 0x10, 0xc1, 0x00};//���÷��͵�ַ
uint32 sendCount     = 0;                             //���ͼ���
uint32 sendOkCount   = 0;                             //���ͳɹ�����
#else
uint8 recvAddress[5] = {0x34, 0xC3, 0x10, 0xc1, 0x00};//���ý��յ�ַ�����շ�ֻҪ֪���Լ��ĵ�ַ����
uint32 recvCount     = 0;                             //���ռ���
#endif

#ifndef WLS_NRF_SEND
void callBackNrfRecv(uint8* dataBuf, uint16 dataBufSize)
{
    ceDebugOp.printf((char*)dataBuf);          //ͨ�����ڴ�ӡ����
    recvCount++;
    ceDebugOp.printf( "Recvount=%u.", recvCount);//��ӡ���ռ���
}
#endif

int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���

    while(ceWlsNrfOp.initial(&myWlsNrf, R7Spi, R2TI2c) != CE_STATUS_SUCCESS)//ʹ��R7SPI��R2TI2c��ʼ��CeWlsNrfģ�飬���ȴ���ģ���������ӳ�ʼ���ɹ���
    {
        ceDebugOp.printf("CeWlsNrf initial return %s!\n", ceSystemOp.getErrorMsg(ceStatus));
        ceSystemOp.delayMs(100);
    };

#ifndef WLS_NRF_SEND
    ceStatus = ceWlsNrfOp.recv(&myWlsNrf, 0, recvAddress, callBackNrfRecv);//����ģʽ�£�ע����ջص��������ȴ�
    if(ceStatus != CE_STATUS_SUCCESS)
    {
        ceDebugOp.printf("CeWlsNrf recv return %s\n", ceSystemOp.getErrorMsg(ceStatus));
    }
#endif

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����

#ifdef WLS_NRF_SEND
        ceDebugOp.sprintf((char*)datBuffer, "SendCount=%u.", sendCount);//׼���������ݣ����ݵĳ��Ȳ��ܳ��� CE_WLS_NRF_PACKET_LENGTH
        ceStatus = ceWlsNrfOp.send(&myWlsNrf, sendAddress, datBuffer, CE_WLS_NRF_PACKET_LENGTH);//�������ݲ���ȡ����״̬
        if (ceStatus == CE_STATUS_SUCCESS)
            sendOkCount++;                      //��¼���ͳɹ��Ĵ���
        sendCount ++;                           //���ͼ�������
        ceDebugOp.printf("SendData: %s SendtCount: %u SendOkCount: %u Status: %s.\n", datBuffer, sendCount, sendOkCount, ceSystemOp.getErrorMsg(ceStatus));//��ӡ���͵����ݺͷ��͵�״̬
#endif
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/

