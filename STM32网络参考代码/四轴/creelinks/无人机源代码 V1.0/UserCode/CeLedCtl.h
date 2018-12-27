/**
  ******************************************************************************
  * @file    CeLedCtl.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeLedCtlģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LED_CTL_H__
#define __CE_LED_CTL_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#include "CeLed1C.h"
     
/*
 *ö�٣������ĸ�LED����˸��ʽ
 */
typedef enum 
{
    CE_LED_CTL_MODE_OFF = 0x00,             /*!< ����LED������״̬*/
    CE_LED_CTL_MODE_IN_CFG,                 /*!< ���ڳ�ʼ������������״̬*/
    CE_LED_CTL_MODE_IN_NORMAL,              /*!< ��������״̬*/
    CE_LED_CTL_MODE_IN_ERROR,               /*!< ����״̬*/
    CE_LED_CTL_MODE_FLASH_CYCLE_P,          /*!< ƫ����*/
    CE_LED_CTL_MODE_FLASH_CYCLE_N,          /*!< ƫ����*/
    CE_LED_CTL_MODE_GOTO_FRONT,             /*!< ��*/
    CE_LED_CTL_MODE_GOTO_BACK,              /*!< ��*/
    CE_LED_CTL_MODE_GOTO_LEFT,              /*!< �󷭹�*/
    CE_LED_CTL_MODE_GOTO_RIGHT,             /*!< �ҷ���*/
}CE_LED_CTL_MODE;


/*
 *CeLedCtl���Զ���
 */
typedef struct
{
    CeLed1C ceLed0;                    
    CeLed1C ceLed1;                  
    CeLed1C ceLed2;                 
    CeLed1C ceLed3;   
    CE_LED_CTL_MODE ctlMode;
    int16 tick;
    CeTicker ceTicker;
}CeLedCtl;
/*
 *CeLedCtl��������
 */
typedef struct
{
    CE_STATUS   (*initial)(CE_RESOURCE ceGpioM0,CE_RESOURCE ceGpioM1,CE_RESOURCE ceGpioM2,CE_RESOURCE ceGpioM3);    /*!< @brief CeLedCtlģ���ʼ��
                                                                                                                         @param ceGpioM0-3:�ĸ�LEDʹ�õ�Gpio��Դ��*/

    void        (*setMode)(CE_LED_CTL_MODE ctlMode);                                                                /*!< @brief �����ĸ�LED��˸�ķ�ʽ
                                                                                                                         @param ctlMode:�ĸ�LED��˸�ķ�ʽ*/

    CE_LED_CTL_MODE (*getMode)(void);                                                                               /*!< @brief ��ȡ��ǰ�ĸ�LED��˸�ķ�ʽ
                                                                                                                         @return ��ǰ�ĸ�LED��˸�ķ�ʽ*/
}CeLedCtlOp;
/*
 *CeLedCtl��������ʵ��
 */
extern const CeLedCtlOp ceLedCtlOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LED_CTL_H__
