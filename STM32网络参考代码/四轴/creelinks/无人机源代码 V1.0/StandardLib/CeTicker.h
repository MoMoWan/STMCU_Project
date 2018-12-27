/**
  ******************************************************************************
  * @file    CeTicker.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   ������ͷ�ļ����������о�ȷ�����ص���صĲ���
  ******************************************************************************
  * @attention
  *
  *1)��ʱ����С���Ϊ1ms����Щ������ƽ̨����Ϊ����ֵ��,���ȸ��ݲ�ͬ�Ĵ�����ƽ̨����ͬ������Ϊ����10us���ڡ�
  *2)����޲���ϵͳ��ƽ̨��ע��Ķ�ʱ������������ϵͳ�ж��ڣ�����в���ϵͳ��ƽ̨����������һ�������ȼ����߳��ڣ��ʾ������ڻص������ڽ����ڽ��к�ʱ������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TICKER_H__
#define __CE_TICKER_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  �ṹ�壬Ticker����������Լ���
  */
typedef struct CeTickerBase
{
    uint32      ID;                                         /*!< ��ʱ������ID*/
    uint32      intervalMs;                                 /*!< ��ʱ����ʱ���*/
    void*       pAddPar;                                    /*!< ��ָ��*/
    void        (*callBack)(void* pAddPar);                 /*!< ���ﶨʱʱ�����Ҫִ�еĺ���*/
    struct      CeTickerBase* nextCeTicker;                 /*!< ����������һ����ʱ����ṹ��*/

    CeExTickerPar  ceExTickerPar;                           /*!< �봦����ƽ̨����ж������*/
}CeTicker;

/**
  * @brief  �ṹ�壬Ticker������ò�������
  */
typedef struct
{
    CE_STATUS   (*registerTicker)(CeTicker* ceTicker);      /*!< @brief ע��һ����ʱ����
                                                                 @param ceTicker:��ʱ��ָ��*/

    CE_STATUS   (*start)(CeTicker* ceTicker);               /*!< @brief ��ʼһ����ʱ����
                                                                 @param ceTicker:��ʱ��ָ��*/

    CE_STATUS   (*stop)(CeTicker* ceTicker);                /*!< @brief ֹͣһ����ʱ����
                                                                 @param ceTicker:��ʱ��ָ��*/

    CE_STATUS   (*unRegister)(CeTicker* ceTicker);          /*!< @brief ȡ��ע�ᶨʱ����
                                                                 @param ceTicker:��ʱ��ָ��*/

    CE_STATUS   (*callBySystem)(void);                      /*!< @brief ��ϵͳ�ڶ�ʱ���ж��ڵ��õĺ���*/
}CeTickerOp;
extern const CeTickerOp ceTickerOp;                     /*!< ������Ticker��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TICKER_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ע��һ���δ�ʱ��������ÿ500ms����һ�λص�����
******************************************************************************
#include "Creelinks.h"
CeTicker myTicker;                              //����Ticker���Զ���
void tickCallBack(void* pAddPar)
{
    ceDebugOp.printf("Tick is running, ID=%d\n", ((CeTicker*)(pAddPar))->ID);
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                        //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myTicker.ID = 0x01;                         //ָ��Ticker��ID��
    myTicker.intervalMs = 500;                  //���õ����ڼ��
    myTicker.pAddPar = &myTicker;               //�������Զ����п�ָ��ָ���Լ�����Ϊ�ص���������Ĳ���
    myTicker.callBack = tickCallBack;           //ָ���ص�����
    ceTickerOp.registerTicker(&myTicker);       //ע��һ��Ticker�δ�ʱ������
    ceTickerOp.start(&myTicker);                //��ʼ������
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ��
        //TODO:���ڴ˴������û�����
    };
}
******************************************************************************
*/
