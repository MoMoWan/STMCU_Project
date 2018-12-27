/**
  ******************************************************************************
  * @file    CeLed1C.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeLed1Cģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LED_1C_H__
#define __CE_LED_1C_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_LED_1C_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_LED_1C_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_LED_1C_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeLed1C.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else
/*
 *CeLed1C���Զ���
 */
typedef struct
{
    CeGpio      ceGpio;                                             /*!< ģ��ʹ�õ���Gpio��Դ����*/
    CeTicker    ceTicker;                                           /*!< CeLed1C��˸�õ��ĵδ���붨ʱ���������*/
    CePwm       cePwm;                                              /*!< ģ��ʹ�õ���Pwm��Դ����*/
    int32       breathAdd;                                          /*!< ģ�鹤���ں�����ģʽ�µ��м����*/
    uint8       isBreathUp;                                         /*!< ģ�鹤���ں�����ģʽ�µ��м����*/
    uint8       ledMode;                                            /*!< ��־ģ�鹤������˸ģʽ�£����Ǻ���ģʽ��*/
    uint8       isGpio;                                             /*!< ��־ʹ�õ���Pwm��Դ������Gpio��Դ*/
    uint16      flashUpMs;                                          /*!< ģ����˸�ķ������ʱ��*/
    uint16      falshDownMs;                                        /*!< ģ����˸��Ϩ�����ʱ��*/
}CeLed1C;

/*
 *CeLed1C��������
 */
typedef struct
{
    void    (*initialByGpio)(CeLed1C* ceLed1C,CE_RESOURCE ceGpio);  /*!< @brief CeLed1Cģ���ʼ��
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��
                                                                         @param ceGpio:CeLed1Cģ��ʹ�õ���Դ��*/

    void    (*initialByPwm)(CeLed1C* ceLed1C, CE_RESOURCE cePwm);   /*!< @brief CeLed1Cģ���ʼ��
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��
                                                                         @param ceGpio:CeLed1Cģ��ʹ�õ���Դ��*/

    void    (*setOn)(CeLed1C* ceLed1C);                             /*!< @brief ����Led״̬Ϊ��
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��*/

    void    (*setOff)(CeLed1C* ceLed1C);                            /*!< @brief ����Led״̬Ϊ��
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��
                                                                         @param ceGpio:CeLed1Cģ��ʹ�õ���Դ��*/

    void    (*setFlash)(CeLed1C* ceLed1C, uint16 flashUpMs,uint16 flashDownMs);/*!<
                                                                         @brief ����Led״̬Ϊ��˸
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��
                                                                         @param flashUpMs:�������ʱ��
                                                                         @param flashDownMs:Ϩ�����ʱ��*/

    void    (*setBreath)(CeLed1C* ceLed1C, uint16 flashMs);         /*!< @brief ����LedΪ����״̬,ֻ��ʹ��Pwm��Դ�ӿڽ��г�ʼ�����ܹ�ʵ�ִ˹���
                                                                         @param ceLed1C:CeLed1C���Զ���ָ��
                                                                         @param flashMs:�������ڣ������һ���������ʱ����*/
}CeLed1COpBase;
/*
 *CeLed1C��������ʵ��
 */
extern const CeLed1COpBase ceLed1COp;

#endif //(__CE_CREELINKS_VERSION__ < __CE_LED_1C_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LED_1C_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ʹled��1HZ��Ƶ����˸
******************************************************************************
#include "Creelinks.h"
#include "CeLed1C.h"
CeLed1C myLed;                                  //����CeLed1C�ṹ��
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceLed1COp.initialByGpio(&myLed, R1AGP);     //ʹ��R1AGP��Դ������ʼ��myLed
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û���
        ceLed1COp.setOn(&myLed);                //����Led״̬Ϊ����
        ceSystemOp.delayMs(500);                //��ʱ500ms
        ceLed1COp.setOff(&myLed);               //����Led״̬ΪϨ��
        ceSystemOp.delayMs(500);                //��ʱ500ms
    };
}
******************************************************************************
*/
