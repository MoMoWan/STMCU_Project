/**
  ******************************************************************************
  * @file    CeBeepNSrc.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBeepNSrcģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BEEP_NSRC_H__
#define __CE_BEEP_NSRC_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BEEP_NSRC_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__ 1                               /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeBeepNSrc.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else
/*
 *CeBeepNSrc���Զ���
 */
typedef struct
{
    CeGpio ceGpio;                                                          /*!< */
}CeBeepNSrc;
/*
 *CeBeepNSrc��������
 */
typedef struct
{
    CE_STATUS   (*initialByGpio)(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE ceGpio);   /*!< @brief CeBeepNSrcģ���ʼ����ʹ��Gpio��Դ
                                                                                     @param ceBeepNSrc:CeBeepNSrc���Զ���ָ��
                                                                                     @param ceGpio:CeBeepNSrcģ��ʹ�õ���Դ��*/

    CE_STATUS   (*initialByPwm)(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE cePwm);     /*!< @brief CeBeepNSrcģ���ʼ����ʹ��Pwm��Դ
                                                                                     @param ceBeepNSrc:CeBeepNSrc���Զ���ָ��
                                                                                     @param cePwm:CeBeepNSrcģ��ʹ�õ���Դ��*/

    void        (*say)(CeBeepNSrc* ceBeepNSrc,uint16 sayMs,uint16 sleepMs, uint8 beepTimes);
                                                                                /*!< @brief CeBeepNSrc��������������Դ�������߳̽��ᱻ������ֱ���������
                                                                                     @param ceBeepNSrc:CeBeepNSrc���Զ���ָ��
                                                                                     @param durationMs:����ʱ�䣬��λ����
                                                                                     @param sleepMs:ֹͣ����ʱ��
                                                                                     @param beepTimes:��������*/

}CeBeepNSrcOp;
/*
 *CeBeepNSrc��������ʵ��
 */
extern const CeBeepNSrcOp ceBeepNSrcOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BEEP_NSRC_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function xxxxxzzzz
******************************************************************************
#include "Creelinks.h"
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                        //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����

    };
}
******************************************************************************
*/
