/**
  ******************************************************************************
  * @file    CeFilter.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ������š�����ƽ����һ�ס����ס���Ԫ�����������˲�����̬���㣩����
  ******************************************************************************
  * @attention
  *
  *1)�����˲�������������initial�н��г�ʼ��
  *2)Ĭ��ʹ�ÿ������˲���ʽ
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_FILTER_H__
#define __CE_FILTER_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#include "CePackage.h"

/*
  *�ṹ�壬��̬�ǽṹ��
  */
typedef struct
{
    fp32 roll;          /*!< ������*/
    fp32 picth;         /*!< ������*/
    fp32 yaw;           /*!< ƫ����*/
    fp32 altitude;      /*!< ���θ߶�*/
    fp32 accelerator;   /*!< �������½������ţ���ΧCE_IMU_MIX_DRIVER_POWER~CE_IMU_MAX_DRIVER_POWER*/
}CeAngles;
/**
  * @brief  �ṹ�壬���ٶ�
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}CeAcc;
/**
  * @brief  �ṹ�壬�����ǽ��ٶ�
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}CeGyr;


#define CE_SLIDE_FILTER_SIZE    20//����ƽ���˲����鳤��
/**
  * @brief  �ṹ�壬����ƽ���˲����Զ���
  */
typedef struct
{
    fp32 array[CE_SLIDE_FILTER_SIZE];
}CeFilterSlider;
/**
  * @brief  �ṹ�壬����ƽ���˲���������
  */
typedef struct
{
    void    (*initial)(CeFilterSlider* ceSlidefilter);              /*!< @brief  ����ƽ���˲���ʼ��
                                                                         @param  ceSlidefilter:����ƽ���˲����Զ���*/

    fp32    (*filter)(CeFilterSlider* ceFilterSlider, fp32 newVal); /*!< @brief  ������ֵ�������˲����ֵ
                                                                         @param  ceSlidefilter:����ƽ���˲����Զ���
                                                                         @param  newVal:δ�����˲�����ֵ
                                                                         @return �˲����ֵ*/
}CeFilterSliderOp;
/**
  * @brief  �ṹ�壬����ƽ���˲�����������
  */
extern const CeFilterSliderOp ceFilterSliderOp;



/*
 * @brief  ë���˲�������
 */
typedef struct
{
    fp32 lastVal;                   /*!< ��һ�ε�ֵ*/
    fp32 maxAbs;                    /*!< ������������������ֵ*/
    fp32 coes[10];                  /*!< Ȩֵ����*/
    uint16 index;                 
    uint16 isUp;                   
}CeFilterBase;

typedef struct
{
    void(*initial)(CeFilterBase* ceFilterBase,fp32 maxAbs);             /*!< @brief  ë���˲�����ʼ��
                                                                             @param  ceFilterBase:ë���˲������Զ���
                                                                             @param  maxAbs:��������֮�����ֵ�����ֵ*/

    fp32(*filter)(CeFilterBase* ceFilterBase, fp32 newVal);             /*!< @brief  �����ݽ���ë���˲���������ֵ�����ֵ
                                                                             @param  ceFilterBase:ë���˲������Զ���
                                                                             @param  newVal:�²ɼ����Ĵ��˲���ֵ
                                                                             @return �˲����ֵ*/  
}CeFilterBaseOp;

extern CeFilterBaseOp ceFilterBaseOp;

/**
  * @brief  �ṹ�壬һ���˲���������
  */
typedef struct
{
    fp32 K1;
    fp32 angle;
}CeFilterYijie;

/**
  * @brief  �ṹ�壬һ���˲���������
  */
typedef struct
{
    void    (*initial)(CeFilterYijie* ceFilterYijie);               /*!< @brief  һ���˲���ʼ��
                                                                         @param  ceFilterYijie:һ���˲����Զ���*/

    fp32    (*filter)(CeFilterYijie* ceFilterYijie, fp32 angle_m, fp32 gyro_m,fp32 dt);/*!< 
                                                                         @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
                                                                         @param  ceFilterYijie:һ���˲����Զ���
                                                                         @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕ�
                                                                         @param  gyro_m:δ�˲��Ľ��ٶ�
                                                                         @return �˲���ĽǶ�ֵ*/
}CeFilterYijieOp;
/**
  * @brief  �ṹ�壬һ���˲�����������
  */
extern const CeFilterYijieOp ceFilterYijieOp;




/**
  * @brief  �ṹ�壬�����˲����Զ���
  */
typedef struct
{
    fp32 K2;
    fp32 angle;
    fp32 x1;
    fp32 x2;
    fp32 y1;
}CeFilterErjie;
/**
  * @brief  �ṹ�壬�����˲���������
  */
typedef struct
{
    void    (*initial)(CeFilterErjie* ceFilterErjie);               /*!< @brief  �����˲���ʼ��
                                                                         @param  ceFilterErjie:�����˲��˲����Զ���*/


    fp32    (*filter)(CeFilterErjie* ceFilterErjie, fp32 angle_m, fp32 gyro_m,fp32 dt);/*!< 
                                                                         @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
                                                                         @param  ceFilterErjie:�����˲����Զ���
                                                                         @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕ�
                                                                         @param  gyro_m:δ�˲��Ľ��ٶ�
                                                                         @return �˲���ĽǶ�ֵ*/
}CeFilterErjieOp;
/**
  * @brief  �ṹ�壬�����˲�����������
  */
extern const CeFilterErjieOp ceFilterErjieOp;





/**
  * @brief  �ṹ�壬��Ԫ��+�����˲����Զ����й���Ԫ���뻥���˲��Ĺ�ϵ����ο�CREELINKS����ĵ�
  */
typedef struct
{
    fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
    fp32 kp;
    fp32 ki;
}CeFilterIMU;
/**
  * @brief  �ṹ�壬��Ԫ��+�����˲�����������
  */
typedef struct
{
    void    (*initial)(CeFilterIMU* ceFilterIMU);                   /*!< @brief  ��Ԫ��+�����˲���ʼ��
                                                                         @param  ceFilterIMU:��Ԫ��+�����˲����Զ���*/

    void    (*filter)(CeFilterIMU* ceFilterIMU, CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle,fp32 halfT);/*!< 
                                                                         @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
                                                                         @param  ceFilterIMU:��Ԫ��+�����˲����Զ���
                                                                         @param  nowAcc:��ǰ���˻����ٶ�����
                                                                         @param  ceNowGyr:��ǰ���˻����ٶ�����
                                                                         @param  ceNowAngle:��ǰ���˻���̬���ݣ���Ԫ����̬�Ǽ�����Ϻ���޸Ĵ�ָ���е�����*/
}CeFilterIMUOp;
/**
  * @brief  �ṹ�壬��Ԫ��+��������������
  */
extern const CeFilterIMUOp ceFilterIMUOp;




/**
  * @brief  �ṹ�壬�������˲����Զ�����ϸ��ο�CREELINKS����ĵ�
  */
typedef struct
{
    fp32 angle;
    fp32 angle_dot; 
    fp32 P[2][2];
    fp32 Pdot[4];
    fp32 Q_angle;
    fp32 Q_gyro;
    fp32 R_angle;
    fp32 C_0;
    fp32 q_bias;
    fp32 angle_err;
    fp32 PCt_0;
    fp32 PCt_1;
    fp32 E; 
    fp32 K_0;
    fp32 K_1; 
    fp32 t_0; 
    fp32 t_1;
}CeFilterKalman;
/**
  * @brief  �ṹ�壬�������˲�����������
  */
typedef struct
{
    void    (*initial)(CeFilterKalman* ceFilterKalman);             /*!< @brief  �������˲���ʼ��
                                                                         @param  ceFilterErjie:�������˲����Զ���*/


    void    (*filter)(CeFilterKalman* ceFilterKalman, fp32* angle_m, fp32* gyro_m,fp32 dt);/*!< 
                                                                         @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
                                                                         @param  ceFilterErjie:�������˲����Զ���
                                                                         @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕȣ�������ɺ�Ὣ������д���ָ���ַ
                                                                         @param  gyro_m:δ�˲��Ľ��ٶȣ�������ɺ�Ὣ������д���ָ���ַ*/
}CeFilterKalmanOp;
/**
  * @brief  �ṹ�壬����������������
  */
extern const CeFilterKalmanOp ceFilterKalmanOp;




/*
 *CeFilter���Զ���
 */
typedef struct
{
    uint16          ceFilterType;        /*!< ��ǰ���˻���̬���㼰�˲���ʽ*/

    fp32            dt;

    CeFilterSlider  ceFilterSliderAccX; /*!< X����ٶȵĻ���ƽ���˲���*/
    CeFilterSlider  ceFilterSliderAccY; /*!< Y����ٶȵĻ���ƽ���˲���*/
    CeFilterSlider  ceFilterSliderAccZ; /*!< Z����ٶȵĻ���ƽ���˲���*/

    CeFilterSlider  ceFilterSliderGyrX; /*!< X����ٶȵĻ���ƽ���˲���*/
    CeFilterSlider  ceFilterSliderGyrY; /*!< Y����ٶȵĻ���ƽ���˲���*/
    CeFilterSlider  ceFilterSliderGyrZ; /*!< Z����ٶȵĻ���ƽ���˲���*/

    CeFilterBase    ceFilterBaseAccX;
    CeFilterBase    ceFilterBaseAccY;
    CeFilterBase    ceFilterBaseAccZ;
    CeFilterBase    ceFilterBaseGyrX;
    CeFilterBase    ceFilterBaseGyrY;
    CeFilterBase    ceFilterBaseGyrZ;

    CeFilterYijie   ceFilterYijiePitch; /*!< һ��Pitch���˲���*/
    CeFilterYijie   ceFilterYijieRoll;  /*!< һ��Roll���˲���*/

    CeFilterErjie   ceFilterErjiePitch; /*!< ����Pitch���˲���*/
    CeFilterErjie   ceFilterErjieRoll;  /*!< ����Roll���˲���*/

    CeFilterIMU     ceFilterIMU;        /*!< ��Ԫ��+�����˲���*/

    CeFilterKalman  ceFilterKalmanPitch;/*!< ������Pitch���˲���*/
    CeFilterKalman  ceFilterKalmanRoll; /*!< ������Roll���˲���*/

    CePackageSend*  cePackageSend;
    CePackageRecv*  cePackageRecv;

    CeAngles     ceAnglesZero;              /*!< ����У����̬��㣬��������е���*/
}CeFilter;
/*
 *CeFilter��������
 */
typedef struct
{
    void (*initial)(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv);/*!< 
                                                                             @brief  �˲���ʼ��
                                                                             @param  cePackageSend:���ݴ��������ʹ�õĽṹ��
                                                                             @param  cePackageRecv:���ݲ��������ʹ�õĽṹ��*/

    void (*filter)(CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle, fp32 dtS); /*!< 
                                                                             @brief  ���뵱ǰ���ٶȼ����ٶ������㵱ǰ���˻�����̬�ǣ���ϸ�ɲο�CREELINKS����ĵ�
                                                                             @param  nowAcc:��ǰ���˻����ٶ�����
                                                                             @param  ceNowGyr:��ǰ���˻����ٶ�����
                                                                             @param  ceNowAngle:��ǰ���˻���̬���ݣ���Ԫ����̬�Ǽ�����Ϻ���޸Ĵ�ָ���е�����
                                                                             @param  dtS:����ִ�����ڣ���λS*/  
}CeFilterOp;
/*
 *CeFilter��������ʵ��
 */
extern const CeFilterOp ceFilterOp;



#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_FILTER_H__
