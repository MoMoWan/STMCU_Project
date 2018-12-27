/**
  ******************************************************************************
  * @file   Creelinks.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinksƽ̨������ڣ���������ģ�鶼Ҫ������ͷ�ļ���
  ******************************************************************************
  * @attention
  *
  *1)�й��봦����ƽ̨��صĶ��塢����Դ�ŵ�ָ�������ݣ��ɲ鿴CeMcu.h�ļ�
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_CREELINKS_H__
#define __CE_CREELINKS_H__

/*Creelinksƽ̨Ӳ������****************************************/
#define __CE_CREELINKS_VERSION__    1           /*!< ��ǰCreelinks.h�ļ��İ汾��*/

#include "CeMcu.h"                              /*!< �봦����ƽ̨��ص����ݼ��꿪�ء���Դ�Ŷ����*/
#include "CeSystem.h"                           /*!< ��ϵͳ������ݣ�����Դ�ӡ�����ʱ�����RTOS������˳��ٽ�ε�*/
#include "CeGpio.h"                             /*!< Gpio�����������Դ���*/
#include "CeAd.h"                               /*!< Adģ��ת����Դ���*/
#include "CeUart.h"                             /*!< Uart������Դ���*/
#include "CeSpi.h"                              /*!< Spi����������Դ���*/
#include "CeDebug.h"                            /*!< L36ͨ��8080����LCD��ʾ�������*/
#include "CeInt.h"                              /*!< Int�ⲿ�ж���Դ���*/
#include "CePwm.h"                              /*!< Pwm���������Դ���*/
#include "CeDa.h"                               /*!< Da��ģת����Դ���*/
#include "CeTimer.h"                            /*!< Timer�ڲ�Ӳ����ʱ����Դ���*/
#include "CeCcp.h"                              /*!< Ccp��������Դ���*/
#include "CeTicker.h"                           /*!< Ticker 1ms������ڶ�ʱ�����*/
#include "CeTask.h"                             /*!< Task�������*/
#include "CeI2c.h"                              /*!< I2c������Դ���*/
#include "CeFlash.h"                            /*!< ������Ƭ��FLASH��Դ���*/


#endif //__CE_CREELINKS_H__
