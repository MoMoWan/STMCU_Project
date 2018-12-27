/**
  ******************************************************************************
  * @file    CeMD.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   �����������ļ�������0~1000������Ϊת��Ϊ0~100%ռ�ձȵ�PWM���
  ******************************************************************************
  * @attention
  *
  *1)����0~1000������ǿ�ȣ������ӦΪ0~100%��ռ�ձ�
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeMD.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
  * @brief  CeMDģ���ʼ��
  * @param  ceMD:CeMD���Զ���
  * @param  ceXX:CeMDģ��ʹ�õ���Դ��
  * @return ϵͳ״̬��
  */
CE_STATUS ceMD_initial(CeMD *ceMD, CE_RESOURCE cePwm)
{
    ceMD->cePwm.ceResource = cePwm;
    ceMD->cePwm.cycleNs = CE_MD_MAX_PWM_CYCLE_NS;
    #ifdef CE_MD_REVERSE
    ceMD->cePwm.dutyNs = CE_MD_MAX_PWM_DUTY_NS;
    #else
    ceMD->cePwm.dutyNs = CE_MD_MIN_PWM_DUTY_NS;
    #endif
    cePwmOp.initial(&(ceMD->cePwm));
        cePwmOp.resetBit(&(ceMD->cePwm));    
    cePwmOp.start(&(ceMD->cePwm));
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  ����Pwm������ǿ�ȣ�0~1000����Ӧռ�ձ�Ϊ0%~100%
  * @param  ceMD:CeMD���Զ���
  * @param  driverPower:Pwm������ǿ�ȣ�0~1000����Ӧռ�ձ�Ϊ0%~100%
  */
void ceMD_setDriverPower(CeMD *ceMD, uint16 driverPower)
{
    if(driverPower > 1000)
    {
        driverPower = 1000;
    }
    #ifdef CE_MD_REVERSE
    ceMD->cePwm.dutyNs = CE_MD_MAX_PWM_CYCLE_NS - CE_MD_MIN_PWM_DUTY_NS + (uint32)(((uint64)(CE_MD_MAX_PWM_DUTY_NS - CE_MD_MIN_PWM_DUTY_NS)*driverPower)/1000);
    #else
    ceMD->cePwm.dutyNs = CE_MD_MIN_PWM_DUTY_NS + (uint32)(((uint64)(CE_MD_MAX_PWM_DUTY_NS - CE_MD_MIN_PWM_DUTY_NS)*driverPower)/1000);
    #endif
        
    cePwmOp.updata(&(ceMD->cePwm));//����Pwm���
}

/**
  * @brief  ��ʼ��CeMDģ���������
  */
const CeMDOp ceMDOp = {ceMD_initial,ceMD_setDriverPower};


#ifdef __cplusplus
}
#endif //__cplusplus
