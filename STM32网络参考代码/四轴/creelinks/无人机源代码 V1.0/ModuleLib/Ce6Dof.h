/**
  ******************************************************************************
  * @file    Ce6Dof.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������Ce6Dofģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_6_DOF_H__
#define __CE_6_DOF_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_6_DOF_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_6_DOF_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_6_DOF_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�Ce6Dof.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

/**
  * @brief  �ṹ�壬���ٶ�
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}Ce6DofAcceleration;

/**
  * @brief  �ṹ�壬�����ǽ��ٶ�
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}Ce6DofGyroscope;

/*
 *Ce6Dof���Զ���
 */
typedef struct
{
    CeI2cMaster         ceI2cMaster;
    Ce6DofAcceleration  acceleration;
    Ce6DofGyroscope     gyroscope;
    Ce6DofGyroscope     gyroscopeZero;
}Ce6Dof;

/*
 *Ce6Dof��������
 */
typedef struct
{
    CE_STATUS              (*initial)(Ce6Dof* Ce6Dof, CE_RESOURCE ceI2c);/*!<
                                                                      @brief Ce6Dofģ���ʼ��
                                                                      @param Ce6Dof:Ce6Dof���Զ���ָ��
                                                                      @param ceI2cMaster:Ce6Dofģ��ʹ�õ���Դ��*/

    Ce6DofAcceleration*    (*getAcceleration)(Ce6Dof* Ce6Dof);   /*!< @brief ��ȡ���ٶ�
                                                                      @param Ce6Dof:Ce6Dof���Զ���ָ��*/

    Ce6DofGyroscope*       (*getGyroscope)(Ce6Dof* Ce6Dof);      /*!< @brief ��ȡ�����ǽ��ٶ�
                                                                      @param Ce6Dof:Ce6Dof���Զ���ָ��*/
}Ce6DofOp;
/*
 *Ce6Dof��������ʵ��
 */
extern const Ce6DofOp ce6DofOp;

#endif //(__CE_CREELINKS_VERSION__ < __CE_6_DOF_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_6_DOF_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ��ȡ6����Ϣ����ͨ��Uart����λ������ʾ
******************************************************************************
#include "Creelinks.h"
#include "Ce6Dof.h"
Ce6Dof my6Dof;                                //�������Զ���
Ce6DofAcceleration* acceleration;              //����������ٶȲ���ָ��
Ce6DofGyroscope* gyroscope;                    //�����������ݲ���ָ��
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ce6DofOp.initial(&my6Dof,I2cx);
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        acceleration = ce6DofOp.getAcceleration(&my6Dof);
        ceDebugOp.printf("Acceleration: x=%d, y=%d, z=%d\n", acceleration->x, acceleration->y,acceleration->z);

        gyroscope = ce6DofOp.getGyroscope(&my6Dof);
        ceDebugOp.printf("Gyroscope: x=%d, y=%d, z=%d\n", gyroscope->x, gyroscope->y,gyroscope->z);

        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/



