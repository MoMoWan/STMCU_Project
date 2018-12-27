/**
  ******************************************************************************
  * @file     CePC33V.h
  * @author   Creelinks Application Team
  * @version  V1.0.0
  * @date    2017-01-06
  * @brief    ������CePC33Vģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PC_33V_H__
#define __CE_PC_33V_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_PC_33V_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_PC_33V_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_PC_33V_VERSION__ > __CE_PC_33V_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CePC33V.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else
/*
 *CePC33V���Զ���
 */
typedef struct
{
    CeAd ceAd;                                                              /*!< Ad��Դ�ӿ����Զ���*/
}CePC33V;
/*
 *CePC33V��������
 */
typedef struct
{
    CE_STATUS   (*initial)(CePC33V* cePC33V, CE_RESOURCE ceAd);             /*!< @brief  CePC33Vģ���ʼ��
                                                                                 @param  cePC33V:CePC33V���Զ���ָ��
                                                                                 @param  ceXX:CePC33Vģ��ʹ�õ���Դ��*/

    fp32        (*getVoltage)(CePC33V* cePC33V);                            /*!< @brief  ��õ�ѹֵ
                                                                                 @param  cePC33V:CePC33V���Զ���ָ��
                                                                                 @return �ɼ����ĵ�ѹֵ*/
}CePC33VOpBase;

/*
 *CePC33V��������ʵ��
 */
extern const CePC33VOpBase cePC33VOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_PC_33V_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__CE_PC_33V_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function �ɼ���ѹ����ͨ�����ڽ���Ϣ��ʾ����λ��
******************************************************************************
#include "Creelinks.h"
#include "CePC33V.h"
CePC33V myPc;
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    cePC33VOp.initial(&myPc,R1AGP);             //ʹ��R1AGP��Ad���ܹ��ܳ�ʼ��ģ��
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û���
        ceDebugOp.printf("The voltage is:%fV\n",cePC33VOp.getVoltage(&myPc));
        ceSystemOp.delayMs(500);
    };
}
******************************************************************************
*/
