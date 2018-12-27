/**
  ******************************************************************************
  * @file    CePC33V.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������PC_33Vģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CePC33V.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  CePC33Vģ���ʼ��
  * @param  cePC33V:CePC33V���Զ���
  * @param  ceXX:PC_33Vģ��ʹ�õ���Դ��
  * @return ϵͳ״̬��
  */
CE_STATUS cePC33V_initial(CePC33V* cePC33V, CE_RESOURCE ceAd)
{
    cePC33V->ceAd.ceResource = ceAd;
    ceAdOp.initial(&(cePC33V->ceAd));
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  ��õ�ѹֵ
  * @param  cePC33V:CePC33V���Զ���
  * @return  �ɼ����ĵ�ѹֵr
  */
fp32 cePC33V_getVoltage(CePC33V* cePC33V)
{
    fp32 temp = ceAdOp.getConvertValue(&(cePC33V->ceAd));
    return (temp / CE_AD_CONVERT_MAX_VAL) * CE_AD_CONVERT_REF_VCC * 10;        
}

const CePC33VOpBase cePC33VOp = {cePC33V_initial, cePC33V_getVoltage};

#ifdef __cplusplus
}
#endif //__cplusplus
