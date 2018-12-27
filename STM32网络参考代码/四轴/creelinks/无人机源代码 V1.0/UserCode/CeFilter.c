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
#include "CeFilter.h"
#include "Math.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  ����ƽ���˲���ʼ��
  * @param  ceSlidefilter:����ƽ���˲����Զ���
  */
void ceFilterSlider_initial(CeFilterSlider* ceFilterSlider)
{
    int i;
    for(i=0;i<CE_SLIDE_FILTER_SIZE;i++)
    {
        ceFilterSlider->array[i]= 0.0f;
    }
}
/**
  * @brief  ������ֵ�������˲����ֵ
  * @param  ceSlidefilter:����ƽ���˲����Զ���
  * @param  newVal:δ�����˲�����ֵ
  * @return �˲����ֵ
  */
fp32 ceFilterSlider_filter(CeFilterSlider* ceFilterSlider,fp32 newVal)
{
    fp32 sum = 0;
    int i;    
    for(i=0;i<CE_SLIDE_FILTER_SIZE-1;i++)
    {
        ceFilterSlider->array[i] = ceFilterSlider->array[i+1];
    }
    ceFilterSlider->array[CE_SLIDE_FILTER_SIZE-1] = newVal;

    for(i=0;i<CE_SLIDE_FILTER_SIZE;i++)
    {
        sum += ceFilterSlider->array[i];
    }
    return sum/CE_SLIDE_FILTER_SIZE;
}
/**
  * @brief  ��ʼ����ƽ���˲���������
  */
const CeFilterSliderOp ceFilterSliderOp = { ceFilterSlider_initial,ceFilterSlider_filter };


/**
  * @brief  ë���˲�����ʼ��
  * @param  ceFilterBase:ë���˲������Զ���
  * @param  maxAbs:��������֮�����ֵ�����ֵ
  */
void ceFilterBase_initial(CeFilterBase* ceFilterBase, fp32 maxAbs)
{
    ceFilterBase->lastVal = 0;
    ceFilterBase->index = 0;
    ceFilterBase->isUp = 0;
    ceFilterBase->maxAbs = maxAbs;
    ceFilterBase->coes[0] = 0.00f;
    ceFilterBase->coes[1] = 0.00f;
    ceFilterBase->coes[2] = 0.00f;
    ceFilterBase->coes[3] = 0.00f;
    ceFilterBase->coes[4] = 0.00f;
    ceFilterBase->coes[5] = 0.01f;
    ceFilterBase->coes[6] = 0.02f;
    ceFilterBase->coes[7] = 0.03f;
    ceFilterBase->coes[8] = 0.04f;
    ceFilterBase->coes[9] = 0.05f;
}
/**
  * @brief  �����ݽ���ë���˲���������ֵ�����ֵ
  * @param  ceFilterBase:ë���˲������Զ���
  * @param  newVal:�²ɼ����Ĵ��˲���ֵ
  * @return �˲����ֵ
  */
fp32 ceFilterBase_filter(CeFilterBase* ceFilterBase, fp32 newVal)
{
    if (ceMathOp.abs(newVal - ceFilterBase->lastVal) > ceFilterBase->maxAbs)
    {
        fp32 n = 0;
        if (newVal >= 0)
        {
            if (ceFilterBase->isUp == 0x00)
            {
                ceFilterBase->index = 0;
            }
            ceFilterBase->isUp = 0x01;
            n = ceFilterBase->lastVal + ceMathOp.abs(ceFilterBase->lastVal - newVal) * ceFilterBase->coes[ceFilterBase->index];
        }
        else
        {
            if (ceFilterBase->isUp == 0x01)
            {
                ceFilterBase->index = 0;
            }
            ceFilterBase->isUp = 0x00;
            n = ceFilterBase->lastVal - ceMathOp.abs(newVal - ceFilterBase->lastVal) * ceFilterBase->coes[ceFilterBase->index];
        }
        ceFilterBase->index++;
        if (ceFilterBase->index >= 10)
        {
            ceFilterBase->index = 0;
            ceFilterBase->lastVal = newVal;
            return newVal;
        }
        return n;
    }
    else
    {
        ceFilterBase->lastVal = newVal;
        ceFilterBase->index = 0;
        return newVal;
    }
}

CeFilterBaseOp ceFilterBaseOp = {ceFilterBase_initial,ceFilterBase_filter};


/**
  * @brief  һ���˲���ʼ��
  * @param  ceFilterYijie:һ���˲����Զ���
  */
void ceFilterYijie_Yinitial(CeFilterYijie* ceFilterYijie)
{
    ceFilterYijie->K1 = 0.05f;
    ceFilterYijie->angle = 0.0f;
}
/**
  * @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
  * @param  ceFilterYijie:һ���˲����Զ���
  * @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕ�
  * @param  gyro_m:δ�˲��Ľ��ٶ�
  * @return �˲���ĽǶ�ֵ
  */
fp32 ceFilterYijie_filter(CeFilterYijie* ceFilterYijie, fp32 angle_m, fp32 gyro_m,fp32 dt)
{
    ceFilterYijie->angle = ceFilterYijie->K1 * angle_m + (1 - ceFilterYijie->K1) * (ceFilterYijie->angle + gyro_m * dt);
    return ceFilterYijie->angle;
}
/**
* @brief  ��ʼ��һ���˲���������
*/
const CeFilterYijieOp ceFilterYijieOp = { ceFilterYijie_Yinitial ,ceFilterYijie_filter};




/**
  * @brief  �����˲���ʼ��
  * @param  ceFilterErjie:�����˲��˲����Զ���
  */
void ceFilterErjie_initial(CeFilterErjie* ceFilterErjie)
{
    ceFilterErjie->K2 = 0.05f;
    ceFilterErjie->angle = 0.0f;
    ceFilterErjie->x1 = 0.0f;
    ceFilterErjie->x2 = 0.0f;
    ceFilterErjie->y1 = 0.0f;
}
/**
  * @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
  * @param  ceFilterErjie:�����˲����Զ���
  * @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕ�
  * @param  gyro_m:δ�˲��Ľ��ٶ�
  * @return �˲���ĽǶ�ֵ
  */
fp32 ceFilterErjie_filter(CeFilterErjie* ceFilterErjie, fp32 angle_m, fp32 gyro_m,fp32 dt)
{
    ceFilterErjie->x1 = (angle_m - ceFilterErjie->angle)*(1 - ceFilterErjie->K2)*(1 - ceFilterErjie->K2);
    ceFilterErjie->y1 = ceFilterErjie->y1 + ceFilterErjie->x1*dt;
    ceFilterErjie->x2 = ceFilterErjie->y1 + 2 * (1 - ceFilterErjie->K2)*(angle_m - ceFilterErjie->angle) + gyro_m;
    ceFilterErjie->angle = ceFilterErjie->angle + ceFilterErjie->x2*dt;
    return ceFilterErjie->angle;
}
/**
  * @brief  ��ʼ�������˲���������
  */
const CeFilterErjieOp ceFilterErjieOp = { ceFilterErjie_initial,ceFilterErjie_filter };




/**
  * @brief  ��Ԫ��+�����˲���ʼ��
  * param  ceFilterIMU:��Ԫ��+�����˲����Զ���
  */
void ceFilterIMU_initial(CeFilterIMU* ceFilterIMU)
{
    ceFilterIMU->q0 = 1;
    ceFilterIMU->q1 = 0;
    ceFilterIMU->q2 = 0;
    ceFilterIMU->q3 = 0;

    ceFilterIMU->kp = 3.000f;
    ceFilterIMU->ki = 0.002f;
}
/**
  * @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
  * @param  ceFilterIMU:��Ԫ��+�����˲����Զ���
  * @param  nowAcc:��ǰ���˻����ٶ�����
  * @param  ceNowGyr:��ǰ���˻����ٶ�����
  * @param  ceNowAngle:��ǰ���˻���̬���ݣ���Ԫ����̬�Ǽ�����Ϻ���޸Ĵ�ָ���е�����
  */
void ceFilterIMU_filter(CeFilterIMU* ceFilterIMU, CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle,fp32 halfT)
{
    float  norm;
    float  vx, vy, vz;
    float  ex, ey, ez;
    float ax, ay, az;
    float gx, gy, gz;

    float q0 = ceFilterIMU->q0;
    float q1 = ceFilterIMU->q1;
    float q2 = ceFilterIMU->q2;
    float q3 = ceFilterIMU->q3;

    float  q0q0 = q0*q0;
    float  q0q1 = q0*q1;
    float  q0q2 = q0*q2;
    //float  q0q3 = q0*q3;//�еشŴ������󣬲Ż�ʹ�õ�������
    float  q1q1 = q1*q1;
    //float  q1q2 = q1*q2;//�еشŴ������󣬲Ż�ʹ�õ�������
    float  q1q3 = q1*q3;
    float  q2q2 = q2*q2;
    float  q2q3 = q2*q3;
    float  q3q3 = q3*q3;
    float  exInt = 0, eyInt = 0, ezInt = 0;

    ax = nowAcc->x;
    ay = nowAcc->y;
    az = nowAcc->z;

    gx = ceNowGyr->x * 0.0174533;//�Ƕ�ת����
    gy = ceNowGyr->y * 0.0174533;
    gz = ceNowGyr->z * 0.0174533;

    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    exInt = exInt + ex * ceFilterIMU->ki;
    eyInt = eyInt + ey * ceFilterIMU->ki;
    ezInt = ezInt + ez * ceFilterIMU->ki;

    gx = gx + ceFilterIMU->kp*ex + exInt;
    gy = gy + ceFilterIMU->kp*ey + eyInt;
    gz = gz + ceFilterIMU->kp*ez + ezInt;

    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    ceFilterIMU->q0 = q0;
    ceFilterIMU->q1 = q1;
    ceFilterIMU->q2 = q2;
    ceFilterIMU->q3 = q3;

    ceNowAngle->picth = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.2957795f; // pitch
    ceNowAngle->roll = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.2957795f; // roll    
    //ceNowAngle->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.2957795f;//δʹ�õشŴ�����������£�ƫ���ǻ���ƫ�ƣ����в��Դ˽ǽ��н���

    ceNowGyr->x = gx/0.0174533;
    ceNowGyr->y = gy/0.0174533;
    ceNowGyr->z = gz/0.0174533;

    //ceIMU.anglesNow.picth = -atan(ax / sqrt(ay*ay + az*az))*57.2957795f;
    //ceIMU.anglesNow.roll = -atan(ay / sqrt(ax*ax + az*az))*57.2957795f;
}
/**
  * @brief  ��ʼ����Ԫ��+������������
  */
const CeFilterIMUOp ceFilterIMUOp = {ceFilterIMU_initial,ceFilterIMU_filter};



/**
  * @brief  �������˲���ʼ��
  * @param  ceFilterErjie:�������˲����Զ���
  */
void ceFilterKalman_initial(CeFilterKalman* ceFilterKalman)
{
    ceFilterKalman->angle = 0;
    ceFilterKalman->angle_dot = 0;

    ceFilterKalman->P[0][0] = 1;
    ceFilterKalman->P[0][1] = 0;
    ceFilterKalman->P[1][0] = 0;
    ceFilterKalman->P[1][1] = 1;

    ceFilterKalman->Pdot[0] = 0;
    ceFilterKalman->Pdot[1] = 0;
    ceFilterKalman->Pdot[2] = 0;
    ceFilterKalman->Pdot[3] = 0;

    ceFilterKalman->R_angle =  0.020f;
    ceFilterKalman->Q_angle = 0.006f;  //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
    ceFilterKalman->Q_gyro = 0.010f;

    ceFilterKalman->C_0 = 1;

    ceFilterKalman->q_bias = 0;//������Ҫ��ʼ��Ϊ0��
    ceFilterKalman->angle_err = 0;
    ceFilterKalman->PCt_0 = 0;
    ceFilterKalman->PCt_1 = 0;
    ceFilterKalman->E = 0;
    ceFilterKalman->K_0 = 0;
    ceFilterKalman->K_1 = 0;
    ceFilterKalman->t_0 = 0;
    ceFilterKalman->t_1 = 0;
}
/**
  * @brief  ������ֵ�������˲���ĽǶ�ֵ����ϸ�ɲο�CREELINKS����ĵ�
  * @param  ceFilterErjie:�������˲����Զ���
  * @param  angle_m:δ�˲����ɼ��ٶ�ֱ�ӻ�ȡ����̬�Ƕȣ�������ɺ�Ὣ������д���ָ���ַ
  * @param  gyro_m:δ�˲��Ľ��ٶȣ�������ɺ�Ὣ������д���ָ���ַ
  */
void ceFilterKalman_filter(CeFilterKalman* ceFilterKalman,fp32* angle_m, fp32* gyro_m,fp32 dt)
{
    ceFilterKalman->angle += (*gyro_m - ceFilterKalman->q_bias) * dt;
    ceFilterKalman->angle_err = *angle_m - ceFilterKalman->angle;

    ceFilterKalman->Pdot[0] = ceFilterKalman->Q_angle - ceFilterKalman->P[0][1] - ceFilterKalman->P[1][0];
    ceFilterKalman->Pdot[1] = -ceFilterKalman->P[1][1];
    ceFilterKalman->Pdot[2] = -ceFilterKalman->P[1][1];
    ceFilterKalman->Pdot[3] = ceFilterKalman->Q_gyro;
    ceFilterKalman->P[0][0] += ceFilterKalman->Pdot[0] * dt;
    ceFilterKalman->P[0][1] += ceFilterKalman->Pdot[1] * dt;
    ceFilterKalman->P[1][0] += ceFilterKalman->Pdot[2] * dt;
    ceFilterKalman->P[1][1] += ceFilterKalman->Pdot[3] * dt;
    ceFilterKalman->PCt_0 = ceFilterKalman->C_0 * ceFilterKalman->P[0][0];
    ceFilterKalman->PCt_1 = ceFilterKalman->C_0 * ceFilterKalman->P[1][0];
    ceFilterKalman->E = ceFilterKalman->R_angle + ceFilterKalman->C_0 * ceFilterKalman->PCt_0;
    ceFilterKalman->K_0 = ceFilterKalman->PCt_0 / ceFilterKalman->E;
    ceFilterKalman->K_1 = ceFilterKalman->PCt_1 / ceFilterKalman->E;
    ceFilterKalman->t_0 = ceFilterKalman->PCt_0;
    ceFilterKalman->t_1 = ceFilterKalman->C_0 * ceFilterKalman->P[0][1];
    ceFilterKalman->P[0][0] -= ceFilterKalman->K_0 * ceFilterKalman->t_0;
    ceFilterKalman->P[0][1] -= ceFilterKalman->K_0 * ceFilterKalman->t_1;
    ceFilterKalman->P[1][0] -= ceFilterKalman->K_1 * ceFilterKalman->t_0;
    ceFilterKalman->P[1][1] -= ceFilterKalman->K_1 * ceFilterKalman->t_1;
    ceFilterKalman->angle += ceFilterKalman->K_0 * ceFilterKalman->angle_err; //���ŽǶ�
    ceFilterKalman->q_bias += ceFilterKalman->K_1 * ceFilterKalman->angle_err;

    ceFilterKalman->angle_dot = *gyro_m - ceFilterKalman->q_bias;//���Ž��ٶ�

    *angle_m = ceFilterKalman->angle;
    *gyro_m = ceFilterKalman->angle_dot;
}
/**
  * @brief ��ʼ����������������
  */
const CeFilterKalmanOp ceFilterKalmanOp = {ceFilterKalman_initial,ceFilterKalman_filter};

/**
  * @brief ȫ�ֱ���
  */
CeFilter ceFilter;
/**
  * @brief ����cePackageRecv�е����ݣ�ʵʱ���˲����㷨�Ĳ������и��µ���
  */
void ceFilter_updataFilterParment(void)
{
    if((ceFilter.ceFilterType & ceFilter.cePackageRecv->status) == 0)//���л��˲��㷨
    {
        if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_YIJIEHUBU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_YIJIEHUBU;
            ceFilterYijieOp.initial(&(ceFilter.ceFilterYijiePitch));
            ceFilterYijieOp.initial(&(ceFilter.ceFilterYijieRoll));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ERJIEHUBU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_ERJIEHUBU;
            ceFilterErjieOp.initial(&(ceFilter.ceFilterErjiePitch));
            ceFilterErjieOp.initial(&(ceFilter.ceFilterErjieRoll));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_IMU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_IMU;
            ceFilterIMUOp.initial(&(ceFilter.ceFilterIMU));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_KALMAN) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_KALMAN;
            ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanPitch));
            ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanRoll));
        }
    }

		if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ANGLE_ZERO) != 0)
    {
        ceFilter.ceAnglesZero.picth = (fp32)(ceFilter.cePackageRecv->zeroPitch) / 1000;
        ceFilter.ceAnglesZero.roll = (fp32)(ceFilter.cePackageRecv->zeroRoll) / 1000;
        ceFilter.ceAnglesZero.yaw = (fp32)(ceFilter.cePackageRecv->zeroYaw) / 1000;
    }
		
    if((ceFilter.cePackageRecv->status & CE_FILTER_IN_DEBUG) == 0)
        return;

    if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_YIJIEHUBU) != 0)//������ڲ�������״̬�������IMU���˲����Ĳ���
    {
        ceFilter.ceFilterYijiePitch.K1 = (fp32)(ceFilter.cePackageRecv->yijieK1) / 1000;
        ceFilter.ceFilterYijiePitch.K1 = (fp32)(ceFilter.cePackageRecv->yijieK1) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ERJIEHUBU) != 0)
    {
        ceFilter.ceFilterErjiePitch.K2 = (fp32)(ceFilter.cePackageRecv->erjieK2) / 1000;
        ceFilter.ceFilterErjieRoll.K2 = (fp32)(ceFilter.cePackageRecv->erjieK2) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_IMU) != 0)
    {
        ceFilter.ceFilterIMU.kp = (fp32)(ceFilter.cePackageRecv->imuKp) / 1000;
        ceFilter.ceFilterIMU.ki = (fp32)(ceFilter.cePackageRecv->imuKi) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_KALMAN) != 0)
    {
        ceFilter.ceFilterKalmanPitch.R_angle = (fp32)(ceFilter.cePackageRecv->filterR_angle) / 1000;
        ceFilter.ceFilterKalmanPitch.Q_angle = (fp32)(ceFilter.cePackageRecv->filterQ_angle) / 1000;
        ceFilter.ceFilterKalmanPitch.Q_gyro = (fp32)(ceFilter.cePackageRecv->filterQ_gyro) / 1000;
            
        ceFilter.ceFilterKalmanRoll.R_angle = (fp32)(ceFilter.cePackageRecv->filterR_angle) / 1000;
        ceFilter.ceFilterKalmanRoll.Q_angle = (fp32)(ceFilter.cePackageRecv->filterQ_angle)/ 1000;
        ceFilter.ceFilterKalmanRoll.Q_gyro = (fp32)(ceFilter.cePackageRecv->filterQ_gyro) / 1000;
    }

}
/**
  * @brief  �˲���ʼ��
  * @param  cePackageSend:���ݴ��������ʹ�õĽṹ��
  * @param  cePackageRecv:���ݲ��������ʹ�õĽṹ��
  */
void ceFilter_initial(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv)
{
    ceFilter.cePackageSend = cePackageSend;                 //���淢�ʹ�����Խṹ�壬���ڴ�����̬�����м���ֵ�Թ��۲�
    ceFilter.cePackageRecv = cePackageRecv;
    ceFilter.ceFilterType = CE_FILTER_IN_KALMAN;            //Ĭ��Kalman�˲���ʽ
    ceFilter.dt = 0.0f;

    ceFilter.ceAnglesZero.picth = 0;
    ceFilter.ceAnglesZero.roll = 0;
    ceFilter.ceAnglesZero.yaw = 0;

    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccX));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccY));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccZ));

    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrX));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrY));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrZ));

    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccX),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccY),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccZ),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrX),60);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrY),60);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrZ),60);

    ceFilterYijieOp.initial(&(ceFilter.ceFilterYijiePitch));
    ceFilterYijieOp.initial(&(ceFilter.ceFilterYijieRoll));


    ceFilterErjieOp.initial(&(ceFilter.ceFilterErjiePitch));
    ceFilterErjieOp.initial(&(ceFilter.ceFilterErjieRoll));

    ceFilterIMUOp.initial(&(ceFilter.ceFilterIMU));

    ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanPitch));
    ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanRoll));
}

/**
  * @brief  ���뵱ǰ���ٶȼ����ٶ������㵱ǰ���˻�����̬�ǣ���ϸ�ɲο�CREELINKS����ĵ�
  * @param  nowAcc:��ǰ���˻����ٶ�����
  * @param  ceNowGyr:��ǰ���˻����ٶ�����
  * @param  ceNowAngle:��ǰ���˻���̬���ݣ���Ԫ����̬�Ǽ�����Ϻ���޸Ĵ�ָ���е�����
  */
void ceFilter_filter(CeAcc* ceNowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle, fp32 dtS)
{
    fp32 norm = 0;
    fp32 ax, ay, az;
    fp32 picthByAcc=0, rollByAcc=0;
		fp32 temp = 0;

    ceFilter_updataFilterParment();//����cePackageRecv�е����ݣ�ʵʱ���˲����㷨�Ĳ������и��µ���
    
    ceFilter.cePackageSend->accX = (int32)(ceNowAcc->x * 1000);//����δ�˲��ļ��ٶ����ݣ��Թ�����վ�۲첨�Σ��Ŵ�100λ����fp32��Int16�ķ�ʽ�������ݴ���
    ceFilter.cePackageSend->accY = (int32)(ceNowAcc->y * 1000);
    ceFilter.cePackageSend->accZ = (int32)(ceNowAcc->z * 1000);
    ceFilter.cePackageSend->gyrX = (int32)(ceNowGyr->x * 1000);//����δ�˲��Ľ��ٶ����ݣ��Թ�����վ�۲첨�Σ�ע��������Ԫ����Kalman�˲��㷨����У�����ٶ�Ư��
    ceFilter.cePackageSend->gyrY = (int32)(ceNowGyr->y * 1000);
    ceFilter.cePackageSend->gyrZ = (int32)(ceNowGyr->z * 1000);

    ceNowAcc->x = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccX), ceNowAcc->x);//�Խ��ٶȽ����޷��˲�ȥë��
    ceNowAcc->y = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccY), ceNowAcc->y);
    ceNowAcc->z = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccZ), ceNowAcc->z);

    ceNowGyr->x = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrX), ceNowGyr->x);//�Լ��ٶȽ����޷��˲�ȥë��
    ceNowGyr->y = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrY), ceNowGyr->y);
    ceNowGyr->z = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrZ), ceNowGyr->z);

    ceNowAcc->x = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccX), ceNowAcc->x);//�Լ��ٶȽ��л���ƽ���˲�
    ceNowAcc->y = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccY), ceNowAcc->y);
    ceNowAcc->z = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccZ), ceNowAcc->z);
       
    norm = sqrt(ceNowAcc->x*ceNowAcc->x + ceNowAcc->y*ceNowAcc->y + ceNowAcc->z*ceNowAcc->z);//���ٶȵ�λ��
    ax = ceNowAcc->x / norm;
    ay = ceNowAcc->y / norm;
    az = ceNowAcc->z / norm;

    picthByAcc = -atan(ax / sqrt(ay*ay + az*az))*57.2957795f;//���ٶ�ֱ�ӽ������̬��
    rollByAcc = -atan(ay / sqrt(ax*ax + az*az))*57.2957795f;

    ceFilter.cePackageSend->pitchByAcc = (int32)(picthByAcc * 1000);//����δ�˲�ǰ�ɼ��ٶ�ֱ�ӽ�������̬�����ݣ��Թ�����վ�۲첨��
    ceFilter.cePackageSend->rollByAcc = (int32)(rollByAcc * 1000);
    ceFilter.cePackageSend->yawByAcc = (int32)(0 * 1000);

    ceFilter.cePackageSend->pitchByGyr = (int16)((ceFilter.cePackageSend->pitchByGyr + ceFilter.dt*ceNowGyr->y) * 100);//����δ�˲�ǰ�ɽ��ٶ�ֱ�ӽ�������̬�����ݣ��Թ�����վ�۲첨��
    ceFilter.cePackageSend->rollByGyr = (int16)((ceFilter.cePackageSend->rollByGyr + ceFilter.dt*ceNowGyr->x) * 100);
    ceFilter.cePackageSend->yawByGyr = (int16)((ceFilter.cePackageSend->yawByGyr + ceFilter.dt*ceNowGyr->z) * 100);

    switch (ceFilter.ceFilterType)
    {
    case CE_FILTER_IN_YIJIEHUBU:
        ceNowAngle->picth = ceFilterYijieOp.filter(&(ceFilter.ceFilterYijiePitch), picthByAcc, ceNowGyr->y,dtS);//����һ�׻��������˻�������̬�������˲�
        ceNowAngle->roll = ceFilterYijieOp.filter(&(ceFilter.ceFilterYijieRoll), rollByAcc, ceNowGyr->x,dtS);
        break;
    case CE_FILTER_IN_ERJIEHUBU:
        ceNowAngle->picth = ceFilterErjieOp.filter(&(ceFilter.ceFilterErjiePitch), picthByAcc, ceNowGyr->y,dtS);//���ö��׻��������˻�������̬�������˲�
        ceNowAngle->roll = ceFilterErjieOp.filter(&(ceFilter.ceFilterErjieRoll), rollByAcc, ceNowGyr->x,dtS);
        break;
    case CE_FILTER_IN_IMU:
        ceNowGyr->x = -ceNowGyr->x;
        ceFilter.cePackageSend->gyrX = -ceFilter.cePackageSend->gyrX;
        ceFilterIMUOp.filter(&(ceFilter.ceFilterIMU), ceNowAcc, ceNowGyr, ceNowAngle,dtS/2);          //������Ԫ��+�����˲������˻�������̬����
        break;
    case CE_FILTER_IN_KALMAN:
        temp = ceNowGyr->x;
        ceFilterKalmanOp.filter(&(ceFilter.ceFilterKalmanRoll), &rollByAcc, &temp,dtS);   
        temp = ceNowGyr->y;
        ceFilterKalmanOp.filter(&(ceFilter.ceFilterKalmanPitch), &picthByAcc, &temp,dtS);  //����Kalman�˲��㷨�����˻���̬���н�����ע�����������Ϊָ�룬�����ڲ��޸�ָ��ֵΪ���¼ƻ��Ľ��
        ceNowAngle->picth = picthByAcc;
        ceNowAngle->roll = rollByAcc;
        break;
    default:
        break;
    }
    ceNowAngle->picth += ceFilter.ceAnglesZero.picth;   //У����̬�ǵ���㣬�������˻�Ư��
    ceNowAngle->roll += ceFilter.ceAnglesZero.roll;
    ceNowAngle->yaw += ceFilter.ceAnglesZero.yaw;

    ceFilter.cePackageSend->accXByFilter = (int32)(ceNowAcc->x*1000);        //���滬���˲���ļ��ٶ����ݣ��Թ�����վ�۲첨��
    ceFilter.cePackageSend->accYByFilter = (int32)(ceNowAcc->y * 1000);
    ceFilter.cePackageSend->accZByFilter = (int32)(ceNowAcc->z * 1000);
    ceFilter.cePackageSend->gyrXByFilter = (int32)(ceNowGyr->x * 1000);      //�������Ư��У������Ľ��ٶ����ݣ��Թ�����վ�۲첨�Σ�ע��������Ԫ����Kalman�˲��㷨����У�����ٶ�Ư��
    ceFilter.cePackageSend->gyrYByFilter = (int32)(ceNowGyr->y * 1000);
    ceFilter.cePackageSend->gyrZByFilter = (int32)(ceNowGyr->z * 1000);
    ceFilter.cePackageSend->pitchByFilter = (int32)(ceNowAngle->picth * 1000);//����Լ��ٶȼ����ٶ����߻�ȡ����̬�ǽ����ں��˲������̬�����ݣ��Թ�����վ�۲첨��
    ceFilter.cePackageSend->rollByFilter = (int32)(ceNowAngle->roll * 1000);
    ceFilter.cePackageSend->yawByFilter = (int32)(0 * 1000);
}
/**
  * @brief  ��ʼ��CeFilter��������ʵ��
  */
const CeFilterOp ceFilterOp = { ceFilter_initial ,ceFilter_filter };

#ifdef __cplusplus
 }
#endif //__cplusplus
