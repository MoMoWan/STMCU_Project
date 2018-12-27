/**
  ******************************************************************************
  * @file    CeJoystick.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeJoystickģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_JOY_STICK_H__
#define __CE_JOY_STICK_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_JOY_STICK_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_JOY_STICK_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_JOY_STICK_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeJoystick.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

#define  CE_JOY_STICK_SLIDER_SIZE   3   /*!< �Բɼ��������ݽ��л���ƽ���˲������*/

typedef struct
{
     int16 x;                           /*!< ��ǰҡ�˵�x����,��Χ-1000 �� 1000*/
     int16 y;                           /*!< ��ǰҡ�˵�y����,��Χ-1000 �� 1000*/
}CeJoystickAxis;

/*
 *CeJoystick���Զ���
 */
typedef struct
{
    CeAd ceAdX;                             /*!< x������ʹ�õ���Ad��Դ*/
    CeAd ceAdY;                             /*!< y������ʹ�õ���Ad��Դ*/
    CeGpio ceGpio;                          /*!< ���ڼ�ⰴť�Ƿ��µ�Gpio��Դ*/
    int32 sliderX[CE_JOY_STICK_SLIDER_SIZE];/*!< X�Ử��ƽ���˲���*/
    int32 sliderY[CE_JOY_STICK_SLIDER_SIZE];/*!< Y�Ử��ƽ���˲���*/
    CeJoystickAxis ceJoystickAxis;          /*!< �����ȡ����������Ϣ*/
    CeJoystickAxis ceCalibrationZero;       /*!< ���У׼���ʱ��������Ϣ*/
}CeJoystick;
/*
 *CeJoystick��������
 */
typedef struct
{
    CE_STATUS       (*initial)(CeJoystick* ceJoystick, CE_RESOURCE ceAd1, CE_RESOURCE ceAd2, CE_RESOURCE ceGpio);/*!<
                                                                         @brief CeJoystickģ���ʼ��
                                                                         @param ceJoystick:CeJoystick���Զ���ָ��
                                                                         @param ceAd1:CeJoystickģ��ʹ�õ�Ad1��Դ��
                                                                         @param ceAd1:CeJoystickģ��ʹ�õ�Ad2��Դ��
                                                                         @param ceAd1:CeJoystickģ��ʹ�õ�Gpio��Դ��*/

    void            (*calibrationZero)(CeJoystick* ceJoystick);     /*!< @brief У׼���
                                                                         @param ceJoystick:CeJoystick���Զ���ָ��
                                                                         @param ceXX:CeJoystickģ��ʹ�õ���Դ��*/

    CeJoystickAxis* (*getAxis)(CeJoystick* ceJoystick);             /*!< @brief ��ȡ����
                                                                         @param ceJoystick:CeJoystick���Զ���ָ��
                                                                         @param ceXX:CeJoystickģ��ʹ�õ���Դ��*/

    uint8           (*getBtnStatus)(CeJoystick* ceJoystick);        /*!< @brief ��ȡ��ť��״̬
                                                                         @param ceJoystick:CeJoystick���Զ���ָ��*/


}CeJoystickOp;
/*
 *CeJoystick��������ʵ��
 */
extern const CeJoystickOp ceJoystickOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_JOY_STICK_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_JOY_STICK_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ��ң�˵�ǰ��λ������ͨ��Uart����λ������ʾ
******************************************************************************
#include "Creelinks.h"
#include "CeJoystick.h"
CeJoystick myJoystick;                          //����ҡ�����Զ���
CeJoystickAxis* joystickAxis;                   //����ҡ��������ʱ�������ָ��
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceJoystickOp.initial(&myJoystick, R1AGP, R5ACGPW, R3GI);
    ceJoystickOp.calibrationZero(&myJoystick);
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        joystickAxis = ceJoystickOp.getAxis(&myJoystick);//��ȡ��ǰҡ����������
        ceDebugOp.printf("Joystick Axis is: x=%d, y=%d\n",joystickAxis->x,joystickAxis->y);
        ceDebugOp.printf("Joystick btn status is: %d\n",ceJoystickOp.getBtnStatus(&myJoystick));
        ceSystemOp.delayMs(100);
    };
}
******************************************************************************
*/
