/**
  ******************************************************************************
  * @file   CeUart.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2016-08-05
  * @brief  Creelinksƽ̨Uart��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)�����Զ��̲߳��������ͬһ��Uart��Դ��������Ҫ�������ͬʱ����
  *2)�й�ÿ��Uart���ռ������жϵ����ȼ�������CeUart.c�ļ��еĺ궨��������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_UART_H__
#define __CE_UART_H__

#include "CeMcu.h"
#include "CeExTra.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  ö�٣�Uart��������
  */
typedef enum
{
    CE_UART_BAUD_RATE_2400 = (uint32)2400,            /*!< ������2400*/
    CE_UART_BAUD_RATE_4800 = (uint32)4800,            /*!< ������4800*/
    CE_UART_BAUD_RATE_9600 = (uint32)9600,            /*!< ������9600*/
    CE_UART_BAUD_RATE_19200 = (uint32)19200,          /*!< ������19200*/
    CE_UART_BAUD_RATE_38400 = (uint32)38400,          /*!< ������38400*/
    CE_UART_BAUD_RATE_43000 = (uint32)43000,          /*!< ������4300*/
    CE_UART_BAUD_RATE_56000 = (uint32)56000,          /*!< ������56000*/
    CE_UART_BAUD_RATE_57600 = (uint32)57600,          /*!< ������576000*/
    CE_UART_BAUD_RATE_115200 = (uint32)115200,        /*!< ������115200*/
}CE_UART_BAUD_RATE;

/**
  * @brief  ö�٣�Uart��������λ
  */
typedef enum
{
    CE_UART_WORD_LENGTH_8B = 0x01,     /*!< ����λ8λ*/
    CE_UART_WORD_LENGTH_9B,            /*!< ����λ9λ*/
}CE_UART_WORD_LENGTH;

/**
  * @brief  ö�٣�Uart����ֹͣλ
  */
typedef enum
{
    CE_UART_STOP_BITS_1 = 0x01,        /*!< 1λֹͣλ*/
    CE_UART_STOP_BITS_0_5,             /*!< ��λֹͣλ*/
    CE_UART_STOP_BITS_2,               /*!< 2λֹͣλ*/
    CE_UART_STOP_BITS_1_5,             /*!< 1λ��λֹͣλ*/
}CE_UART_STOP_BITS;

/**
  * @brief  ö�٣�Uart������żУ��λ
  */
typedef enum
{
    CE_UART_PARITY_NO = 0x01,          /*!< ����żУ��λ*/
    CE_UART_PARITY_EVEN,               /*!< żУ��λ*/
    CE_UART_PARITY_ODD,                /*!< ��У��λ*/
}CE_UART_PARITY;

/**
  * @brief  �ṹ�壬Uart����������Լ���
  */
typedef struct
{
    CE_RESOURCE             ceResource;                 /*!< Uart��Ӧ����Դ��*/
    CE_UART_BAUD_RATE       uartBaudRate;               /*!< Uart�Ĳ�����*/
    CE_UART_WORD_LENGTH     uartWordLength;             /*!< Uart�����ݿ��*/
    CE_UART_STOP_BITS       uartStopBits;               /*!< Uart��ֹͣλ*/
    CE_UART_PARITY          uartParity;                 /*!< Uart����żУ��λ*/

    uint8*                  recvBuf;                    /*!< Uart���ջ���*/
    uint16                  recvBufSize;                /*!< Uart���ջ��������*/
    void*                   pAddPar;

    CeFifo                  ceExFifo;                   /*!< Uart����ʹ�õĻ���Fifo*/
    CeExUartPar             ceExUartPar;                /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeUart;

/**
  * @brief  �ṹ�壬Uart������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeUart* ceUart);             /*!< @brief ��ʼ��Uart
                                                             @param ceUart:ceUart���Զ���ָ��*/

    void        (*start)(CeUart* ceUart);               /*!< @brief ��ʼUart
                                                             @param ceUart:ceUart���Զ���ָ��*/

    CE_STATUS   (*sendData)(CeUart* ceUart,uint8* dataBuf, uint16 dataBufSize);/*!<
                                                             @brief ��������
                                                             @param ceUart:ceUart���Զ���ָ��
                                                             @param dataBuf:�����͵�����
                                                             @param dataBufSize:�����͵����ݳ���*/

    uint16      (*getRecvDataCount)(CeUart* ceUart);    /*!< @brief ��ý��ջ����еĿ�������
                                                             @param ceUart:ceUart���Զ���ָ��
                                                             @return ���ؿɶ�ȡ�����ݳ���*/

    uint16      (*readData)(CeUart* ceUart, uint8* dataBuf, uint16 readCount);/*!<
                                                             @brief ��������
                                                             @param ceUart:ceUart���Զ���ָ��
                                                             @param dataBuf:���յ������ݴ�ŵ�λ��
                                                             @param readCount:Ҫ��ȡ�����ݸ���
                                                             @return ����ʵ�ʶ�ȡ�������ݳ���*/

    void        (*stop)(CeUart* ceUart);                /*!< @brief ֹͣUart
                                                             @param ceUart:ceUart���Զ���ָ��*/

    void        (*clearRecvBuf)(CeUart* ceUart);        /*!< @brief ��ս��ջ���
                                                             @param ceUart:ceUart���Զ���ָ��*/
}CeUartOp;
extern const CeUartOp ceUartOp;                         /*!< ������Uart��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_UART_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ʹ��Uart����λ��ͨѶ������λ�������������ݣ���ԭ�����͸���λ��
******************************************************************************
#include "Creelinks.h"
#define UART_RECV_BUF_SIZE  1024                    //Uart���ջ����С
CeUart myUart;                                      //����Uart���Զ���
uint8 recvBuf[UART_RECV_BUF_SIZE];                  //Uart���ջ�������
int main(void)
{
    ceSystemOp.initial();                           //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                      //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myUart.ceResource = R18Uart;                     //ָ��Uartʹ�õ���Դ��
    myUart.recvBuf = recvBuf;                       //ָ��Uart���ջ���
    myUart.recvBufSize = UART_RECV_BUF_SIZE;        //ָ��Uart���ջ����С
    myUart.uartBaudRate = CE_UART_BAUD_RATE_115200; //ָ��Uart�Ĺ�������Ϊ115200
    myUart.uartWordLength = CE_UART_WORD_LENGTH_8B; //ָ��Uart���ݳ���Ϊ8bit
    myUart.uartStopBits = CE_UART_STOP_BITS_1;      //ָ��Uart��ֹͣλΪ1
    myUart.uartParity = CE_UART_PARITY_NO;          //ָ��Uart����ż����λΪ��
    ceUartOp.initial(&myUart);                      //��ʼ��Uart
    ceUartOp.start(&myUart);                        //��ʼUart�����뷢��
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks������ѭ��
        //TODO:���ڴ˴������û�����
        while (ceUartOp.getRecvDataCount(&myUart) > 0)//���Uart�Ƿ��յ�����
        {
            uint8 recvTemp[128];                    //���建�棬���ڶ�ȡ���յ�������
            uint16 tureRecvCount = ceUartOp.readData(&myUart, recvTemp, 128);//��ȡ���յ�������
            ceUartOp.sendData(&myUart, recvTemp, tureRecvCount);             //����ȡ�������ݣ���ͨ��Uart�ڷ�����λ��
        }
        ceSystemOp.delayMs(10);                     //��ʱ10ms
    };
}
******************************************************************************
*/
