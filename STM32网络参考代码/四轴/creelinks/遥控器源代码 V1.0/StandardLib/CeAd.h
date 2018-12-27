/**
  ******************************************************************************
  * @file   CeAd.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinksƽ̨Ad����Ĳ���ͷ�ļ�,�����йش�����ƽ̨Ad���ݵ���ز���
  ****************************************************************************** 
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_AD_H__
#define __CE_AD_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  �ṹ�壬AD����������Լ���
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< Ad��Ӧ����Դ��*/

    CeExAdPar       ceExPar;                        /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeAd;

/**
  * @brief  �ṹ�壬AD������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeAd* ceAd);             /*!< @brief ��ʼ��Adת��
                                                         @param ceAd:Ad���Զ���ָ��*/

    uint32      (*getConvertValue)(CeAd* ceAd);     /*!< @brief ���Adת�����
                                                         @param ceAd:Ad���Զ���ָ��
                                                         @return ADת�����*/
}CeAdOp;
extern const CeAdOp ceAdOp;                         /*!< ������Ad��صĲ���*/

#endif //__CE_USE_AD__

#ifdef __cplusplus
 }
#endif //__cplusplus

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ��ȡAd��ת��ֵ����ͨ��Uart�ڴ�������ڵ�������
******************************************************************************
#include "Creelinks.h"
CeAd myAd;                                      //����Ad���Զ���
uint32 convertVal;                              //ת������������ʱ����
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                   //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myAd.ceResource = PAxA;                     //����Ad��Դ��
    ceAdOp.initial(&myAd);
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����       
        ceDebugOp.printf("ConvertVal = %d",ceAdOp.getConvertValue(&myAd)); //��ӡAdת�����
        ceSystemOp.delayMs(500);                //��ʱ500ms
    };
}
******************************************************************************
*/


