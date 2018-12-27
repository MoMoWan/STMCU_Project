/**
  ******************************************************************************
  * @file    CeCcp.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   Creelinksƽ̨�ⲿ���������CeCcp���������������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_CCP_H__
#define __CE_CCP_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  �ṹ�壬CCP����������Լ���
  */
typedef struct
{
    CE_RESOURCE ceResource;                             /*!< Ccp��Ӧ����Դ��*/
    uint32      ceCntVal;                               /*!< �û��趨��Ccp�����ٽ�ֵ*/
    void*       pAddPar;                                /*!< ��ָ�룬�����ڴ��ݶ������*/
    void        (*callBackReachCntVal)(void* pAddPar);  /*!< Ccp���������û����õ��ٽ�ֵ����Ҫִ�еĻص�����*/

    CeExCcpPar  ceExCcpPar;                             /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeCcp;

/**
  * @brief  �ṹ�壬CCP������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeCcp* ceCcp);               /*!< @brief ��ʼ��Ccp������
                                                             @param ceCcp:ceCcp���Զ���ָ��*/

    void        (*start)(CeCcp* ceCcp);                 /*!< @brief ��ʼCcp����
                                                             @param ceCcp:ceCcp���Զ���ָ��*/

    void        (*stop)(CeCcp* ceCcp);                  /*!< @brief ֹͣCcp����
                                                             @param ceCcp:ceCcp���Զ���ָ��*/

    uint32      (*getNowCcpCnt)(CeCcp* ceCcp);          /*!< @brief ��õ�ǰCcp������ֵ����ֵһ��С�ڵ���ceMaxCnt
                                                             @param ceCcp:ceCcp���Զ���ָ��
                                                             @return ��ȡ���μ������ڵļ���ֵ*/

    uint32      (*getAllCcpCnt)(CeCcp* ceCcp);          /*!< @brief ��ôӿ�ʼ������(����startCcpʱ��ʼ)��������һ���ļ���ֵ
                                                             @param ceCcp:ceCcp���Զ���ָ��
                                                             @return ��ȡ�ӿ�ʼ������ĿǰΪֹ�ܵļ���ֵ*/

    void        (*clearCcpCnt)(CeCcp* ceCcp);           /*!< @brief �����������0��ʼ���¼���
                                                             @param ceCcp:ceCcp���Զ���ָ��*/
}CeCcpOp;
extern const CeCcpOp ceCcpOp;                       /*!< ������Ccp��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_CCP_H__
/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function �趨Ccp��������ֵΪ10����ÿ500ms��ȡ��ǰֵ��ͨ��Uart�������λ��
******************************************************************************
#include "Creelinks.h"
CeCcp myCcp;                                       //Ccp���Զ���
uint8 ccpCount;                                    //��ǰCcp�ļ���ֵ

// @brief  Ccp�������ﵽ����ֵ����¼��ص�
// @param  pAddPar:Ccp����ָ�����pAddPar����
void ceCcpReachCntCallBack(void* pAddPar)
{
    ceDebugOp.printf("Ccp is reach setting count!\n");
}

int main(void)
{
    ceSystemOp.initial();                          //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                     //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myCcp.ceResource = RxC;                        //ָ����������Դ��
    myCcp.ceCntVal = 10;                           //ָ����������ֵ�󣬽����жϻص�
    myCcp.callBackReachCntVal = ceCcpReachCntCallBack;//ָ���ص�����
    myCcp.pAddPar = &myCcp;                        //ָ�������п�ָ��Ϊ�����жϻص��д����ָ��
    ceCcpOp.initial(&myCcp);                       //��ʼ��������
    ceCcpOp.start(&myCcp);                         //��ʼ����
    while (1)
    {
        ceTaskOp.mainTask();                       //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        ccpCount = ceCcpOp.getNowCcpCnt(&myCcp);   //��õ�ǰ����ֵ
        ceDebugOp.printf("Ccp count is %d\n", ccpCount);//��ӡ��ǰ����ֵ
        ceSystemOp.delayMs(500);                   //��ʱ500ms
    };
}
******************************************************************************
*/



