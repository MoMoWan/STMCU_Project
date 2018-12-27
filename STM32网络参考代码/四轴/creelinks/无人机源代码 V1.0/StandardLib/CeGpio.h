/**
  ******************************************************************************
  * @file    CeGpio.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeGpio��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)���ܴ�����ƽ̨��Լ������ͬ�Ĵ������в�ͬ��IO������ģʽ��Ϊ��֤�����ԣ�Creelinks������û����õ�IO��ģʽ�Զ�ƥ
  *  ���Ӧ��������IO��ģʽ
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_GPIO_H__
#define __CE_GPIO_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  �ṹ�壬GPIO����������Լ���
  */
typedef struct
{
    CE_RESOURCE     ceResource;                                 /*!< GPIO��Ӧ����Դ��*/
    CE_GPIO_MODE    ceGpioMode;                                 /*!< ���趨��GPIO����ģʽ*/

    CeExGpioPar     ceExGpioPar;                                /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeGpio;

/**
  * @brief  �ṹ�壬GPIO������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeGpio* ceGpio);                     /*!< @brief ��ʼ��һ��GPIO
                                                                     @param ceGpio:GPIO���Զ��󼯺�ָ��*/

    void        (*setBit)(CeGpio* ceGpio);                      /*!< @brief ����GPIO�ڵ�ֵΪ1
                                                                     @param ceGpio:GPIO���Զ��󼯺�ָ��*/

    void        (*resetBit)(CeGpio* ceGpio);                    /*!< @brief ����GPIO�ڵ�ֵΪ0
                                                                     @param ceGpio:GPIO���Զ��󼯺�ָ��*/

    uint8       (*getBit)(CeGpio* ceGpio);                      /*!< @brief ��ȡGPIO�ڵ�ֵ��0x01��0x00
                                                                     @param ceGpio:GPIO���Զ��󼯺�ָ��
                                                                     @return ��ǰGpio�ڵĵ�ƽ״̬*/

    void        (*setMode)(CeGpio* ceGpio,CE_GPIO_MODE ceGpioMode);/*!<
                                                                     @brief ����Gpio�ڵĹ�����ʽ
                                                                     @param ceGpio:GPIO���Զ��󼯺�ָ��
                                                                     @param ceGpioMode:GPIO����ģʽ*/
}CeGpioOp;
extern const CeGpioOp ceGpioOp;                                 /*!< ������GPIO��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_GPIO_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function �趨Gpioÿ500ms����һ�ε�ƽ��ת
******************************************************************************
#include "Creelinks.h"
CeGpio myGpio;                                          //����Gpio���Զ���
int main(void)
{
    ceSystemOp.initial();                               //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                                //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myGpio.ceResource = RxG;                            //ָ��Gpioʹ�õ���Դ��
    myGpio.ceGpioMode = CE_GPIO_MODE_OUT_OD;            //����Gpio�Ĺ���ģʽ
    ceGpioOp.initial(&myGpio);                          //��ʼ��Gpio
    while (1)
    {
        ceTaskOp.mainTask();                            //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        ceGpioOp.setBit(&myGpio);                   //�趨GpioΪ�ߵ�ƽ
        ceDebugOp.printf("Gpio status: up.\n");        //����λ�����������Ϣ
        ceSystemOp.delayMs(500);                        //��ʱ500ms
        ceGpioOp.resetBit(&myGpio);                 //�趨GpioΪ�ߵ�ƽ
        ceDebugOp.printf("Gpio status: down.\n");      //����λ�����������Ϣ
        ceSystemOp.delayMs(500);                        //��ʱ500ms
    };
}
******************************************************************************
*/
