/**
  ******************************************************************************
  * @file    CePwm.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CePwmͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)���������ڸ���������ƽ̨��Creelinks���ܱ�֤���е�Pwm��Դ�������������ڼ�ռ�ձȣ����ִ�����ƽ̨���ܻ���ֶ��
  *  Pwm��Դ���ڱ�����ͬ��Լ������ϸ�ɲ鿴CePwm.c�е�����
  *2)
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PWM_H__
#define __CE_PWM_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  �ṹ�壬Pwm����������Լ���
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< Pwm��Ӧ����Դ��*/
    uint32          cycleNs;                        /*!< Pwm�����ڣ���λns*/
    uint32          dutyNs;                         /*!< Pwm�ĸߵ�ƽʱ�䣬��λns*/
    CeExPwmPar      ceExPwmPar;                     /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CePwm;

/**
  * @brief  �ṹ�壬Pwm������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CePwm* cePwm);           /*!< @brief ��ʼ��Pwm
                                                         @param cePwm:Pwm���Զ���*/

    void        (*start)(CePwm* cePwm);             /*!< @brief ��ʼPwm���
                                                         @param cePwm:Pwm���Զ���*/

    void        (*updata)(CePwm* cePwm);            /*!< @brief ����Pwm����
                                                         @param cePwm:Pwm���Զ���*/

    void        (*stop)(CePwm* cePwm);              /*!< @brief ֹͣPwm���
                                                         @param cePwm:Pwm���Զ���*/

    void        (*setBit)(CePwm* cePwm);            /*!< @brief ����Pwm�ڵ����Ϊ�ߵ�ƽ��ע�⣺ֻ����Pwmû�����ʱ���д˲�����
                                                         @param cePwm:Pwm���Զ���*/

    void        (*resetBit)(CePwm* cePwm);          /*!< @brief ����Pwm�ڵ����Ϊ�͵�ƽ��ע�⣺ֻ����Pwmû�����ʱ���д˲�����
                                                         @param cePwm:Pwm���Զ���*/
}CePwmOp;
extern const CePwmOp cePwmOp;                       /*!< ������Pwm��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif  //__CE_PWM_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ʹ��һ·Pwm�������1Khz�ģ�ռ�ձ�Ϊ25%�ķ���
******************************************************************************
#include "Creelinks.h"
CePwm myPwm;                                    //����Pwm���Զ���
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                        //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myPwm.ceResource = RxP;                     //ָ��Pwm���Զ���ʹ�õ���Դ��
    myPwm.cycleNs = 1000000;                    //ָ��Pwm��������ڣ���λ����
    myPwm.dutyNs = myPwm.cycleNs / 4;           //ָ��Pwm�����ռ�ձȸߵ�ƽ����ʱ�䣬��λ����
    cePwmOp.initial(&myPwm);                    //��ʼ��Pwm���
    cePwmOp.start(&myPwm);                      //��ʼPwm�������
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
    };
}
******************************************************************************
*/
