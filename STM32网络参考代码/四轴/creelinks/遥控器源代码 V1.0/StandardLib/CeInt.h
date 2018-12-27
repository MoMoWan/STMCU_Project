/**
  ******************************************************************************
  * @file    CeInt.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeInt��ͷ�ļ�����Ҫ�����ⲿ�жϵ���Ӧ�봦��
  ******************************************************************************
  * @attention
  *
  *1)��ͬ��Int��Դ���в�ͬ���ж����ȼ����û�����CeInt.c�еĺ궨��������
  *2)ÿ��Int�Ļص������ж��ڽ��У���������ڻص���ִ�к�ʱ����
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_INT_H__
#define __CE_INT_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  ö�٣��ⲿ�ж�Int�Ĵ�����ʽ
  */
typedef enum
{
    CE_INT_MODE_TRIGGER_FALLING = 0x00,             /*!< �жϴ�����ʽ���½��ش���*/
    CE_INT_MODE_TRIGGER_RISING,                     /*!< �жϴ�����ʽ �������ش���*/
}CE_INT_MODE;

/**
  * @brief  �ṹ�壬�ⲿ�ж�Int����������Լ���
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< �ⲿ�ж�Int��Ӧ����Դ��*/
    CE_INT_MODE     ceIntMode;                      /*!< �ⲿ�ж�Int�Ĺ���ģʽ*/
    void*           pAddPar;                        /*!< ��ָ�룬���ݸ��ⲿ�ж�Int����ʱ�Ļص�*/
    void            (*callBack)(void* pAddPar);     /*!< �ж��¼�����ʱ����Ҫִ�еĺ���
                                                         callBack:�ⲿ�жϷ���ʱ����Ҫִ�еĻص���pAddPar:��ָ�룬�����ڴ��Ͷ������*/

    CeExIntPar      ceExIntPar;                     /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeInt;

/**
  * @brief  �ṹ�壬�ⲿ�ж�Int������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeInt* ceInt);           /*!< @brief ��ʼ���ⲿ�ж�Int
                                                         @param ceInt:�ⲿ�ж�Int���Զ���ָ��*/

    void        (*setMode)(CeInt* ceInt, CE_INT_MODE ceIntMode);/*!<
                                                         @brief �����жϷ�ʽ�������������½�
                                                         @param ceInt:�ⲿ�ж�Int���Զ���ָ��
                                                         @param ceIntMode:��Ҫ���õ��жϷ�ʽ*/

    void        (*start)(CeInt* ceInt);             /*!< @brief ��ʼ�ⲿ�ж�Int���
                                                         @param ceInt:�ⲿ�ж�Int���Զ���ָ��*/

    void        (*stop)(CeInt* ceInt);              /*!< @brief ֹͣ�ⲿ�ж�Int���
                                                         @param ceInt:�ⲿ�ж�Int���Զ���ָ��*/

    uint8       (*getBit)(CeInt* ceInt);            /*!< @brief ��ȡ�ⲿ�ж�Int�ڶ�Ӧ��Gpio�ĵ�ƽֵ��0x01��0x00
                                                         @param ceInt:�ⲿ�ж�Int���Զ���ָ��
                                                         @reutrn �ж϶�Ӧ���ŵĵ�ƽ״̬*/
}CeIntOp;
extern const CeIntOp ceIntOp;                       /*!< �������ⲿ��Int����صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_INT_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function �ⲿ�жϲ��Ժ������½��ش���
******************************************************************************
#include "Creelinks.h"
CeInt myInt;

void intEventCallBack(void* pAddPar)
{
    ceDebugOp.printf("Enter the interrupt, Gpio status:%d\n", ceIntOp.getBit((CeInt*)(pAddPar)));//�����ж��¼��󣬴�ӡ������Ϣ
}
int main(void)
{
    ceSystemOp.initial();                           //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                            //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myInt.ceResource = RxI;                         //ָ����Դ�жϹ���ģ��ʹ�õ���Դ��
    myInt.ceMode = CE_INT_MODE_TRIGGER_FALLING;          //ָ���ⲿ�ж����½����ж�
    myInt.pAddPar = &myInt;                         //���ÿ�ָ��ָ���Լ������жϻص���ʹ��
    myInt.callBack = intEventCallBack;              //ָ���¼��ص�����
    ceIntOp.initial(&myInt);                        //��ʼ���ⲿ�ж�
    ceIntOp.start(&myInt);                          //ʹ���ⲿ�ж�
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
    };
}
******************************************************************************
*/
