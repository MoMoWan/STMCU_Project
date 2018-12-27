/**
  ******************************************************************************
  * @file    CeDa.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   Creelinksƽ̨CeDa��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_DA_H__
#define __CE_DA_H__
#include "CeMcu.h"
#ifdef __cplusplus
 extern "C" {
#endif
/**
  * @brief  �ṹ�壬DA����������Լ���
  */
typedef struct
{
    CE_RESOURCE             ceResource;                             /*!< Da��Ӧ����Դ��*/
    uint32                  convertIntervalNs;                      /*!< ����������ҪDaת�������������ʱ�䣬��λns*/
    void*                   pAddPar;                                /*!< ��ָ�룬�����ڴ��ݶ������*/
    void                    (*callBackConvertFinish)(void* pAddPar);/*!< ���һ��DAת������Ҫִ�еĺ���*/

    CeExDaPar               ceExDaPar;                              /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeDa;

/**
  * @brief  �ṹ�壬DA������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeDa* ceDa);                             /*!< @brief ��ʼ��һ��Da
                                                                         @param ceDa:Da���Զ���ָ��*/

    void        (*start)(CeDa* ceDa, const uint16* dataBuf, uint32 dataBufSize);/*!<
                                                                         @brief ��ʼDAת��
                                                                         @param ceDa:Da���Զ���ָ��
                                                                         @param dataBuf:��Ҫת�������ݻ�����
                                                                         @param dataBufSize:��Ҫת�������ݸ�����ӦС�����ݻ���������Ч����*/

    void        (*startFixedVoltage)(CeDa* ceDa, uint16 Val);       /*!< @brief Da����̶�ֵ
                                                                         @param ceDa:Da���Զ���ָ��*/

    void        (*stop)(CeDa* ceDa);                                /*!< @brief ֹͣDaת��
                                                                         @param ceDa:Da���Զ���ָ��*/

    void        (*updata)(CeDa* ceDa);                              /*!< @brief ����Da����
                                                                         @param ceDa:Da���Զ���*/


}CeDaOp;
extern const CeDaOp ceDaOp;                                     /*!< ������Da��صĲ���*/


#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_DA_H__

/**
******************************************************************************
* @brief   ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ͨ��Da���100Hz�����ǲ�
******************************************************************************
#include "Creelinks.h"

#define DA_BUF_SIZE     1000                    //Da��ת�������鳤��
CeDa myDa;                                      //Da���Զ���ָ��
uint16 outVal[DA_BUF_SIZE];                     //Da��ת��������
void daConvertFinishCallBack(void* pAddPar)
{
    ceDaOp.start((CeDa*)(pAddPar), outVal, DA_BUF_SIZE);   //���һ��ת�����ٽ���һ��ת���Դﵽ�������������
}

int main(void)
{
    int i;
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R23Uart);                 //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    for (i = 0; i < DA_BUF_SIZE; i++)
    {
        outVal[i] = i;//�����ʼ��
    }
    myDa.ceResource = R8ADG;                    //ָ����Դ��
    myDa.convertIntervalNs = 10000;             //�趨ÿ������֮���ת�������10000ns*1000���㣽10ms����������ǲ�Ƶ��Ϊ1s/10ms = 100Hz
    myDa.pAddPar = &myDa;                       //���������п�ָ��ָ���Լ����������ֶ����ͬ��Da����
    myDa.callBackConvertFinish = daConvertFinishCallBack;//ָ��ת����ɵĻص�����
    ceDaOp.initial(&myDa);                      //��ʼ��Da
    ceDaOp.start(&myDa, outVal, DA_BUF_SIZE);   //��ʼת��
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
    };
}
******************************************************************************
*/


