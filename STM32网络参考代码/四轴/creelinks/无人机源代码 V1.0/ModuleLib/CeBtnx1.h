/**
  ******************************************************************************
  * @file    CeBtnx1.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBtnx1ģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BTN_X1_H__
#define __CE_BTN_X1_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BTN_X1_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_BTN_X1_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_BTN_X1_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeBtnx1.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else
/*
 *CeBtnx1���Զ���
 */
typedef struct
{
    CeInt       ceInt;                              /*!< ģ��ʹ���ⲿ�ж�Int��ɳ�ʼ���õ����ⲿ�ж�Int��Դ���Զ���*/
    CeGpio      ceGpio;                             /*!< ģ��ʹ��Gpio��ɳ�ʼ���õ���Gpio��Դ���Զ���*/
    CeTicker    ceTicker;                           /*!< ģ��ʹ�õĵδ���붨ʱ�����Զ���*/
    uint8       btnStatus;                          /*!< ��ť��ǰ״̬*/
    void        (*callBackPressEvent)(void);        /*!< �����¼��ص�����*/
}CeBtnx1;

/*
 *CeBtnx1��������
 */
typedef struct
{
    CE_STATUS   (*initialByGpio)(CeBtnx1* ceBtnx1, CE_RESOURCE ceGpio, void (*callBackPressEvent)(void));/*!<
                                                         @brief CeBtnx1ģ��ʹ��Gpio������ɳ�ʼ��
                                                         @param CeBtnx1:CeBtnx1���Զ���ָ��
                                                         @param ceGpio:CeBtnx1ģ��ʹ�õ���Դ��
                                                         @param callBackPressEvent:��������ʱ�Ļص�����������Ҫ�ص���CE_NULL����*/

    CE_STATUS   (*initialByInt)(CeBtnx1* ceBtnx1, CE_RESOURCE ceInt, void (*callBackPressEvent)(void));/*!<
                                                         @brief CeBtnx1ģ��ʹ���ⲿ�ж�Int����ɳ�ʼ��
                                                         @param CeBtnx1:CeBtnx1���Զ���ָ��
                                                         @param ceInt:CeBtnx1ģ��ʹ�õ���Դ��
                                                         @param callBackPressEvent:��������ʱ�Ļص�����������Ҫ�ص���CE_NULL����*/

    uint8       (*getStatus)(CeBtnx1* ceBtnx1);     /*!< @brief ��ȡCeBtnx1״̬������1�����Ѱ��£�����0����δ����
                                                         @param ceBtnx1:CeBtnx1���Զ���ָ��*/

    CE_STATUS   (*waitForPressDown)(CeBtnx1* ceBtnx1, uint32 outTimeMs);/*!<
                                                         @brief �ȴ���������(������ⲿ�ж�Int��ʽ��ʼ���������ʹ�ô˷���)
                                                         @param ceBtnx1:CeBtnx1���Զ���ָ��
                                                         @param outTimeMs:�ȴ��ĳ�ʱʱ�䣬Ms*/

    CE_STATUS   (*waitForPressUp)(CeBtnx1* ceBtnx1, uint32 outTimeMs);/*!<
                                                         @brief �ȴ���������
                                                         @param ceBtnx1:CeBtnx1���Զ���ָ��
                                                         @param outTimeMs:�ȴ��ĳ�ʱʱ�䣬Ms*/
}CeBtnx1OpBase;

/*
 *CeBtnx1��������ʵ��
 */
extern const CeBtnx1OpBase ceBtnx1Op;

#endif //(__CE_CREELINKS_VERSION__ < __CE_BTN_X1_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BTN_X1_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ��ťÿ����һ�Σ�������λ���ϴ�ӡ������Ϣ
******************************************************************************
#include "Creelinks.h"
#include "CeBtnx1.h"
CeBtnx1 myBtn;                                  //����CeBtnx1���Զ���
void callBackPress(void)
{
    ceDebugOp.printf("myBtn is Press down.\n");//���°���ִ���¼��ص�����ӡ������Ϣ
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceBtnx1Op.initialByGpio(&myBtn, R1AGP, callBackPress);  //ʹ����Դ��R1AGP��ʼ��myBtn
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û���

    };
}
******************************************************************************
*/
