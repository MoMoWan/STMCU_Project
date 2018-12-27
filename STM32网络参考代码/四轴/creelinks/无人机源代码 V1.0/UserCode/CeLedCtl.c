/**
  ******************************************************************************
  * @file    CeLedCtl.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeLedCtlģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeLedCtl.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
CeLedCtl ceLedCtl;      /*!< ����ȫ�ֱ���*/
/**
  * @brief  ��ʱ������CeTicker�Ļص���������ϸ�ɲο�CREELINKSƽ̨�й�CeTicker����ĵ�
  * @param  pAddPar:ceLedCtl����ָ��
  */
void ceLedCtl_callBack(void* pAddPar)
{
    if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_FLASH_CYCLE_P)//˳ʱ���Led0��Led3ѭ������
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
        case 22:
        case 32:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 30:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_FLASH_CYCLE_N)//˳ʱ���Led3��Led0ѭ������
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
        case 22:
        case 32:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 30:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_FRONT)//�ɺ���ǰ����
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));

            break;
        case 10:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_BACK)//��ǰ�������
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_LEFT)//������������
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_RIGHT)//������������
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_CFG)//�������ü���ʼ��״̬������
    {
        switch (ceLedCtl.tick)
        {
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 40:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_NORMAL)//����״̬
    {
        switch (ceLedCtl.tick)
        {
        case 2:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 100:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_ERROR)//����״̬������LED����
    {
        switch (ceLedCtl.tick)
        {
        case 2:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));

            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }
    ceLedCtl.tick++;
}

/**
  * @brief  CeLedCtlģ���ʼ��
  * @param  @param ceGpioM0-3:�ĸ�LEDʹ�õ�Gpio��Դ��
  * @return ϵͳ״̬��
  */
CE_STATUS ceLedCtl_initial(CE_RESOURCE ceGpioM0,CE_RESOURCE ceGpioM1,CE_RESOURCE ceGpioM2,CE_RESOURCE ceGpioM3)
{
    ceLedCtl.ctlMode = CE_LED_CTL_MODE_OFF;
    ceLedCtl.tick = 0;

    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed0),ceGpioM0);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed1),ceGpioM1);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed2),ceGpioM2);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed3),ceGpioM3);

    ceLedCtl.ceTicker.callBack = ceLedCtl_callBack;
    ceLedCtl.ceTicker.ID = (uint16)ceGpioM0;
    ceLedCtl.ceTicker.intervalMs = 10;
    ceLedCtl.ceTicker.pAddPar = &ceLedCtl;
    ceTickerOp.registerTicker(&(ceLedCtl.ceTicker)); 
    ceTickerOp.start(&(ceLedCtl.ceTicker));

    return CE_STATUS_SUCCESS;
}

/**
  * @brief �����ĸ�LED��˸�ķ�ʽ
  * @param ctlMode:�ĸ�LED��˸�ķ�ʽ
  */
void ceLedCtl_setMode(CE_LED_CTL_MODE ctlMode)
{
    if(ctlMode == ceLedCtl.ctlMode) return;//��Ҫ���õ�״̬�뵱ǰ״̬��ͬ����ֱ�ӷ���
    ceLedCtl.ctlMode = ctlMode;
    ceLedCtl.tick = 0;  

    ceLed1COp.setOff(&(ceLedCtl.ceLed0));
    ceLed1COp.setOff(&(ceLedCtl.ceLed1));
    ceLed1COp.setOff(&(ceLedCtl.ceLed2));
    ceLed1COp.setOff(&(ceLedCtl.ceLed3));
}
/**
  * @brief ��ȡ��ǰ�ĸ�LED��˸�ķ�ʽ
  * @return ��ǰ�ĸ�LED��˸�ķ�ʽ
  */
CE_LED_CTL_MODE ceLedCtl_getMode()
{
    return ceLedCtl.ctlMode;
}
/**
  * @brief  ��ʼ��CeLedCtlģ���������
  */
const CeLedCtlOp ceLedCtlOp = {ceLedCtl_initial,ceLedCtl_setMode,ceLedCtl_getMode};

#ifdef __cplusplus
 }
#endif //__cplusplus
