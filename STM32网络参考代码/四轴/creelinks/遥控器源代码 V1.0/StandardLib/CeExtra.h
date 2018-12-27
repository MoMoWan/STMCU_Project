/**
  ******************************************************************************
  * @file    CeExtra.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeExtra��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��Creelinks��Ҫ��֤���ļ����ԣ��󲿷�C���Կ⺯������Creelinks�Լ�ʵ��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_EXTRA_H__
#define __CE_EXTRA_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  �ṹ�壬Creelinks����ѧ������صĺ���
  */
typedef struct
{
    fp32 (*abs)(fp32 val);                                    /*!< @brief �Բ���ȡ����ֵ
                                                                     @param val:��Ҫ�����ֵ
                                                                     @return ������*/
}CeMathOp;
extern const CeMathOp ceMathOp;                             /*!< ��������ѧ������صĲ���*/

/**
  * @brief  �ṹ�壬Creelinks���ַ�����صĲ�������
  */
typedef struct
{
    uint32  (*strlen)(const char* str);                         /*!< @brief �����ַ������ȣ�������'\0'
                                                                     @param str:��Ҫ�����ֵ
                                                                     @return �ַ�������*/

    int8    (*strcmp)(const char* str1, const char* str2);      /*!< @brief �Ƚ������ַ����Ƿ����
                                                                     @param str1:�ַ���1
                                                                     @param str2:�ַ���2
                                                                     @return 0��ȣ� 1�����*/

    char *  (*strcpy)(char *dest, char *src);                   /*!< @brief �ַ�������
                                                                     @param dest:Ŀ�ĵ�ַ
                                                                     @param src:Դ��ַ�ַ���
                                                                     @return Ŀ�ĵ�ַ��ָ��*/

    char*   (*itoa)(int32 val, char* str, uint8 radix);         /*!< @brief ������ת��Ϊ�ַ���
                                                                     @param val:����
                                                                     @param str:�ַ�������
                                                                     @param radix:��Ҫת���Ľ��ƣ�8/10/16
                                                                     @return ת����ɺ���ַ���*/

    int32   (*atoi)(const char* str);                           /*!< @brief ���ַ���ת��Ϊ����
                                                                     @param str:Ҫת�����ַ���
                                                                     @return ת���������*/

}CeStringOp;
extern const CeStringOp ceStringOp;                         /*!< �������ַ�����صĲ���*/

/**
  * @brief  �ṹ�壬Fifo�������Լ���
  */
typedef struct
{
    uint8*  buff;                                               /*!< Fifo����*/
    uint16  buffSize;                                           /*!< Fifo��������*/
    uint16  readIndex;                                          /*!< Fifo�ж�����*/
    uint16  writeIndex;                                         /*!< Fifo��д����*/
    uint8   isReadLock;                                         /*!< Fifo����������ֹ����߳�ͬʱ��*/
    uint8   isWriteLock;                                        /*!< Fifoд��������ֹ�������ͬʱд*/
}CeFifo;

/**
  * @brief  �ṹ�壬Creelinks��Fifo��صĲ�������
  */
typedef struct
{
    void    (*initial)(CeFifo* ceFifo);                         /*!< @brief Fifo��ʼ��
                                                                     @param ceFifo:Cefifo��������ָ��*/

    uint8   (*isEmpty)(CeFifo* ceFifo);                         /*!< @brief ���Fifo�Ƿ�Ϊ�գ����Ƿ������ݿɶ�
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @return Fifo�Ƿ�Ϊ�գ�����0��Fifo��Ϊ�գ�����1��FifoΪ��*/

    uint16  (*getCanReadSize)(CeFifo* ceFifo);                  /*!< @brief ��ÿɶ������ݳ���
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @return �ɶ������ݳ���*/

    uint16  (*getCanWriteSize)(CeFifo* ceFifo);                 /*!< @brief ��ÿ�д������ݳ���
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @return ��д������ݳ���*/

    uint16  (*write)(CeFifo* ceFifo, uint8* dataInBuf, uint16 dataInCount);/*!<
                                                                     @brief ��Fifo��д������
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @param dataInBuf:��д������ݻ���
                                                                     @param dataInCount:��д������ݳ���
                                                                     @return ʵ��д������ݳ��ȣ�������Fifo����*/

    uint16  (*read)(CeFifo* ceFifo, uint8* dataOutBuf, uint16 dataOutCount);/*!<
                                                                     @brief ��Fifo�ж�ȡ����
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @param dataOutBuf:��ȡ��������ŵĻ���
                                                                     @param dataOutCount:��Ҫ��ȡ�����ݳ���
                                                                     @return ʵ�ʶ�ȡ�����ݳ���*/

    void    (*clear)(CeFifo* ceFifo);                           /*!< @brief ���Fifo�е�����
                                                                     @param ceFifo:Cefifo��������ָ��*/

    uint8   (*getReadLockStatus)(CeFifo* ceFifo);               /*!< @brief ���Fifo�Ƿ��ڶ�����״̬
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @return ����0��Fifoδ��ʹ�ã�1��Fifo���ڽ��ж���д����*/
    uint8   (*getWriteLockStatus)(CeFifo* ceFifo);              /*!< @brief ���Fifo�Ƿ���д����״̬
                                                                     @param ceFifo:Cefifo��������ָ��
                                                                     @return ����0��Fifoδ��ʹ�ã�1��Fifo���ڽ��ж���д����*/

}CeFifoOp;
extern const CeFifoOp ceFifoOp;

/**
  * @brief  �ṹ�壬˫�����Fifo�������Լ���
  */
typedef struct
{
    CeFifo ceFifoOne;                                           /*!< Fifoһ������*/
    CeFifo ceFifoTwo;                                           /*!< Fifo��������*/
}CeDoubleFifo;

/**
  * @brief  �ṹ�壬Creelinks��˫����Fifo��صĲ�������
  */
typedef struct
{
    void    (*initial)(CeDoubleFifo* ceDoubleFifo);             /*!< @brief Fifo��ʼ��
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��*/

    uint8   (*isEmpty)(CeDoubleFifo* ceDoubleFifo);             /*!< @brief ���Fifo�Ƿ�Ϊ�գ����Ƿ������ݿɶ�
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��
                                                                     @return Fifo�Ƿ�Ϊ��*/

    uint16  (*getCanReadSize)(CeDoubleFifo* ceDoubleFifo);      /*!< @brief ��ÿɶ������ݳ���
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��
                                                                     @return �ɶ������ݳ���*/

    uint16  (*getCanWriteSize)(CeDoubleFifo* ceDoubleFifo);     /*!< @brief ��ÿ�д������ݳ���
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��
                                                                     @return ��д������ݳ���*/

    uint16  (*write)(CeDoubleFifo* ceDoubleFifo, uint8* dataInBuf, uint16 dataInCount);/*!<
                                                                     @brief ��Fifo��д������
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��
                                                                     @param dataInBuf:��д������ݻ���
                                                                     @param dataInCount:��д������ݳ���
                                                                     @return ʵ��д������ݳ��ȣ�������Fifo����*/

    uint16  (*read)(CeDoubleFifo* ceDoubleFifo, uint8* dataOutBuf, uint16 dataOutCount);/*!<
                                                                     @brief ��Fifo�ж�ȡ����
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��
                                                                     @param dataOutBuf:��ȡ��������ŵĻ���
                                                                     @param dataOutCount:��Ҫ��ȡ�����ݳ���
                                                                     @return ʵ�ʶ�ȡ�����ݳ���*/

    void    (*clear)(CeDoubleFifo* ceDoubleFifo);               /*!< @brief ���Fifo�е�����
                                                                     @param ceDoubleFifo:CeDoubleFifo��������ָ��*/

}CeDoubleFifoOp;
extern const CeDoubleFifoOp ceDoubleFifoOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_EXTRA_H__
