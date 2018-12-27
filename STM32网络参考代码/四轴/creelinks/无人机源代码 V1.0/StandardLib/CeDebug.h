/**
  ******************************************************************************
  * @file    CeDeBug.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeDeBugģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_DEBUG_H__
#define __CE_DEBUG_H__

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"

#define CE_DEBUG_RECV_BUF_SIZE      32                  /*!< ���ڵ���ʹ�õ�Uart���ջ����С�����Ҫ����λ�����մ������ݣ����޸Ĵ�ֵ*/
#define CE_DEBUG_APPEND_BUF_SIZE    256                 /*!< AppendString ʹ�õĻ���*/
/*
 *CeDeBug���Զ���
 */
typedef struct
{
    CeUart ceUart;                                      /*!< ���ڵ���ʹ�õ�Uard����*/
    uint8 recvBuf[CE_DEBUG_RECV_BUF_SIZE];              /*!< ����ʹ�õ��Ľ��ջ���*/
    void(*appendString)(const char* msg);               /*!< ����ʱ����Ҫ׷��CMD��Ϣ�Ļص�*/
    char appendStringBuf[CE_DEBUG_APPEND_BUF_SIZE];     /*!< ����׷��CMD��Ϣʱ��sprintfʹ�õĻ���*/
    uint16 appendStringBufIndex;                        /*!< */
    uint8 isPrintfFinish ;                              /*!< */
}CeDebug;
/*
 *CeDeBug��������
 */
typedef struct
{

    CE_STATUS   (*initial)(CE_RESOURCE ceUart);/*!<
                                             @brief ��ʹ�õ�UART���д����ʽ������initial��ִ�д˺�����ceUart:ʹ���ĸ�UART��Դ��Ϊ��ʽ��
                                             @param ceUart:��ӡ������Ϣʹ�õ�Uart��Դ��*/

    void        (*registerAppendString)(void (appendString)(const char* msg));/*!<
                                             @brief ���û�ʹ�ö������ʾ�豸ʱ�����Խ���ʾ�豸��appendStringʹ�ô˺���ע�ᣬע����ɺ�ɴﵽ��ʾ�豸��ʾ������Ϣ������
                                             @param appendString:��Ҫע���appendString����*/

    void        (*unRegisterAppendString)(void);/*!<
                                             @brief ȡ���ڶ�����ʾ�豸����ʾ������Ϣ����*/


    void        (*printf)(const char* msg, ...);/*!<
                                             @brief ͨ��UART�ڴ�ӡ��ʽ��Ϣ�����裬���ڵ���initialDebug�󣬲ſ�������ʹ��
                                             @param msg:���ӡ����Ϣ*/

    int         (*sprintf)(char *buffer, const char *msg, ...);/*!<
                                             @brief ͨ��UART�ڴ�ӡ��ʽ��Ϣ���ڴ�
                                             @param buffer:�ڴ滺��
                                             @param msg:���ӡ����Ϣ*/


    uint8       (*getRecvDataCount)(void);/*!<
                                             @brief ʹ��Uart���е���ʱ�������λ���������Ŀ��õ�����������
                                             @return ���õ��������� */

    uint8       (*getRecvData)(uint8* dataOutBuf, uint8 readCount);/*!<
                                             @brief ʹ��Uart���е���ʱ�������λ���������ĵ�������
                                             @param dataOutBuf:�����ȡ���ݵĻ���
                                             @param readCount:��Ҫ��ȡ�����ݳ���
                                             @return ʵ�ʶ�ȡ�������ݳ���*/

}CeDebugOp;
/*
 *CeDeBug��������ʵ��
 */
extern const CeDebugOp ceDebugOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_DEBUG_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function xxxxxzzzz
******************************************************************************
#include "Creelinks.h"
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                        //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����

    };
}
******************************************************************************
*/
