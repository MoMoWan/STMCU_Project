/**
  ******************************************************************************
  * @file    CeBmp180.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBmp180ģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BMP180_H__
#define __CE_BMP180_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BMP180_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_BMP180_NEED_CREELINKS_VERSION__ 1                               /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_BMP180_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeBmp180.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else



typedef struct
{
    fp32     temperature;        /*!< �¶�ֵ*/
    int32    pressure;           /*!< ��ѹֵ*/
    fp32     altitude;           /*!< ���θ߶�*/
}CeBmp180Environment;

/*
 *CeBmp180���Զ���
 */
typedef struct
{
    CeI2cMaster ceI2cMaster;
    int16       AC1;
    int16       AC2;
    int16       AC3;
    uint16      AC4;
    uint16      AC5;
    uint16      AC6;
    int16       B1;
    int16       B2;
    int16       MB;
    int16       MC;
    int16       MD;
    int32       UT;
    int32       UP;
    CeBmp180Environment environment;
    uint32 lastSystemTimeMs;    /*!< ����Bmp180����һ����ѹ��ȡ��Ҫ����4.5msʱ�䣬��ֵ���ڼ���ӿ�ʼת����*/
    uint8       asyncStep;
    fp32        lastAltiude;

}CeBmp180;
/*
 *CeBmp180��������
 */
typedef struct
{
    CE_STATUS               (*initial)(CeBmp180* ceBmp180, CE_RESOURCE ceI2cMaster);/*!< 
                                                                                 @brief CeBmp180ģ���ʼ��
                                                                                 @param ceBmp180:CeBmp180���Զ���ָ��
                                                                                 @param ceI2cMaster:CeBmp180ģ��ʹ�õ���Դ��*/
                                                                             
    CeBmp180Environment*    (*getEnvironment)(CeBmp180* ceBmp180);          /*!< @brief ��ȡ�Ѿ���У�����¶ȡ���ѹ�����θ߶�
                                                                                 @param ceBmp180:CeBmp180���Զ���ָ��*/

    CeBmp180Environment*    (*getEnvironmentAsync)(CeBmp180* ceBmp180);     /*!< @brief ��ȡ�Ѿ���У�����¶ȡ���ѹ�����θ߶ȡ��ǲ���ϵͳ�������첽���ã��ɱ�֤��10ms�ȴ���ʱ��������ɺ󷵻���ȷ��������򷵻�CE_NULL
                                                                                 @param ceBmp180:CeBmp180���Զ���ָ��*/
}CeBmp180Op;
/*
 *CeBmp180��������ʵ��
 */
extern const CeBmp180Op ceBmp180Op;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BMP180_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BMP180_H__

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
