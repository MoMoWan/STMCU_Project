/**
  ******************************************************************************
  * @file    CePackage.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ������CePackageģ�������ͷ�ļ�,�������ݵĴ���������
  ******************************************************************************
  * @attention
  *
  *1)��Ӷ����������ֱ��CePackageSend��CePackageRecv����Ӽ��ɣ�ϵͳ�Զ�����ṹ�峤��
  *2)��ӵı���һ��Ϊint32�����������޷�Ԥ��Ĺ���
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PACKAGE_H__
#define __CE_PACKAGE_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"

#define  CE_PACKAGE_PACK_SIZE               32  /*!< ������wifi�����ݴ���ģ�鷢��һ�ΰ��ĳ��ȣ�����32byte����ΪNRF24L01+����һ�ΰ����Ⱦ���32���ֽ�*/

#define CE_PACKAGE_SEND_BUF_SIZE (CE_PACKAGE_PACK_SIZE * (sizeof(CePackageRecv)/sizeof(uint32)*4/(CE_PACKAGE_PACK_SIZE-6) + ((sizeof(CePackageRecv)/sizeof(uint32)*4)%(CE_PACKAGE_PACK_SIZE-6) == 0 ?0:1)))   /*!< ���ݷ��ͽṹ���еĲ��������Ͱ�ͷ����Ϣ����������ͻ��泤��*/
#define CE_PACKAGE_RECV_BUF_SIZE (CE_PACKAGE_PACK_SIZE * (sizeof(CePackageRecv)/sizeof(uint32)*4/(CE_PACKAGE_PACK_SIZE-6) + ((sizeof(CePackageRecv)/sizeof(uint32)*4)%(CE_PACKAGE_PACK_SIZE-6) == 0 ?0:1)))   /*!< ���ݽ��սṹ���еĲ��������Ͱ�ͷ����Ϣ����������ջ��泤��*/

#define CE_FILTER_IN_YIJIEHUBU  0x0001          /*!< �Ƿ�ʹ��һ�׻����˲���*/
#define CE_FILTER_IN_ERJIEHUBU  0x0002          /*!< �Ƿ�ʹ�ö��׻����˲���*/
#define CE_FILTER_IN_IMU        0x0004          /*!< �Ƿ�ʹ����Ԫ��+�����˲���*/
#define CE_FILTER_IN_KALMAN     0x0008          /*!< �Ƿ�ʹ�ÿ������˲���*/
#define CE_FILTER_IN_DEBUG      0x0010          /*!< �Ƿ����˲�����������ģʽ��*/
#define CE_PID_IN_DEBUG         0x0020          /*!< �Ƿ���PID��������֮��*/
#define CE_CTL_BTN_LEFT         0x0040          /*!< ��������ҡ�˰�ť*/
#define CE_CTL_BTN_RIGHT        0x0080          /*!< ������ҡ���Ұ�ť*/
#define CE_CTL_BTN_S2A          0x0100          /*!< ��ť����ťS2A*/
#define CE_CTL_BTN_S2B          0x0200          /*!< ��ť����ťS2B*/
#define CE_CTL_BTN_S2C          0x0400          /*!< ��ť����ťS2C*/
#define CE_CTL_BTN_S2D          0x0800          /*!< ��ť����ťS2D*/
#define CE_CTL_TYPE_STATION     0x1000          /*!< ���˻����ܿط�ʽ������վ��*/
#define CE_CTL_TYPE_CTL         0x2000          /*!< ���˻����ܿط�ʽ��ң����*/
#define CE_CTL_TYPE_PHONE       0x4000          /*!< ���˻����ܿط�ʽ���ֻ�*/


/**
  * @brief  �ṹ�壬��Ҫ���͵�����
  */
typedef struct 
{
    int32 leftX;                    /*!< ҡ����/��������ҡ��Xֵ����1000~+1000*/
    int32 leftY;                    /*!< ҡ����/��������ҡ��Yֵ����1000~+1000*/
    int32 rightX;                   /*!< ҡ����/��������ҡ��Xֵ����1000~+1000*/
    int32 rightY;                   /*!< ҡ����/��������ҡ��Yֵ����1000~+1000*/

    int32 status;                   /*!< ״̬���ƣ�ÿ��bit��������ļ��궨��*/               

    int32 outPitchP;                /*!< ͨ������վ��������˻�PID����ʱ�����͵Ĳ���*/
    int32 outPitchI;
    int32 outPitchD;
    int32 inPitchP;
    int32 inPitchI;
    int32 inPitchD;

    int32 outRollP;
    int32 outRollI;
    int32 outRollD;
    int32 inRollP;
    int32 inRollI;
    int32 inRollD;

    int32 yawOutKp;
    int32 yawOutKi;
    int32 yawOutKd;
    int32 yawInKp;
    int32 yawInKi;
    int32 yawInKd;

    int32 yijieK1;                   /*!< ͨ������վ��������˻��˲�����ʱ�����͵Ĳ���*/
    int32 erjieK2;

    int32 imuKp;
    int32 imuKi;

    int32 filterR_angle;
    int32 filterQ_angle;
    int32 filterQ_gyro;

    int32 driverPower0Zero;         /*!< ����У׼�ĸ����*/
    int32 driverPower1Zero;
    int32 driverPower2Zero;
    int32 driverPower3Zero;
}CePackageSend;
/**
  * @brief  �ṹ�壬��Ҫ���յ�����
  */
typedef struct 
{
    int32 pitchByFilter;            /*!< �����ٶȺͽ��ٶ�ֱ�ӽ��������̬�ǽ����ںϺ����ջ�õ���̬��Pitch*/
    int32 rollByFilter;             /*!< �����ٶȺͽ��ٶ�ֱ�ӽ��������̬�ǽ����ںϺ����ջ�õ���̬��Roll*/
    int32 yawByFilter;              /*!< �����ٶȺͽ��ٶ�ֱ�ӽ��������̬�ǽ����ںϺ����ջ�õ���̬��Yaw*/

    int32 temperature;              /*!< �¶�ֵ*/
    int32 altitude;                 /*!< ���θ߶�*/
    int32 batVoltage;               /*!< ��ǰ��صĵ�ѹֵ��ʾ����DC-DC��·��ֱ�Ӷ�﮵�ؽ��в���*/

    int32 accelerator;              /*!< ���˻���ǰ����ǿ��0~1000��ң�������ž������ᴦ���Ľ��*/    
    int32 pressure;                /*!< ��ѹֵ*/

    int32 driverPower0;             /*!< �ĸ������ǰ������ǿ�ȣ�0~1000*/
    int32 driverPower1;
    int32 driverPower2;
    int32 driverPower3;

    int32 accX;                     /*!< δ�˲��ĵ�ǰ���ٶ�����*/
    int32 accXByFilter;             /*!< �����˲���ļ��ٶ����ݣ���ʹ�õ���վ�۲��˲�Ч��*/    
    int32 accY;
    int32 accYByFilter;        
    int32 accZ;
    int32 accZByFilter;            
    int32 gyrX;                     /*!< δ�˲��ĵ�ǰ���ٶ�����*/
    int32 gyrXByFilter;             /*!< ����У����ļ��ٶ����ݣ���ʹ�õ���վ�۲��˲�Ч��*/    
    int32 gyrY;
    int32 gyrYByFilter;            
    int32 gyrZ;
    int32 gyrZByFilter;        

    int32 pitchByAcc;               /*!< �ɼ��ٶ�ֱ�ӽ��������̬��Pitch*/
    int32 pitchByGyr;               /*!< �ɽ��ٶ�ֱ�ӽ��������̬��Pitch*/
    int32 rollByAcc;                /*!< �ɼ��ٶ�ֱ�ӽ��������̬��Roll*/
    int32 rollByGyr;                /*!< �ɽ��ٶ�ֱ�ӽ��������̬��Roll*/
    int32 yawByAcc;                 /*!< �ɼ��ٶ�ֱ�ӽ��������̬��Yaw*/
    int32 yawByGyr;                 /*!< �ɽ��ٶ�ֱ�ӽ��������̬��Yaw*/
}CePackageRecv;
/*
 *CePackage��������
 */
typedef struct
{

    CE_STATUS   (*initialSend)(CePackageSend* cePackageSend);       /*!< @brief cePackageSendģ���ʼ�����Խṹ���е����ݽ�����0����
                                                                         @param cePackageSend:CePackageSend���Զ���ָ��*/

    CE_STATUS   (*initialRecv)(CePackageRecv* cePackageRecv);       /*!< @brief cePackageRecvģ���ʼ�����Խṹ���е����ݽ�����0����
                                                                         @param cePackageRecv:CePackageRecv���Զ���ָ��*/

    uint8*      (*dealSend)(CePackageSend* cePackageSend);          /*!< @brief ��cePackageSend�ṹ����д�������ش�����ֱ�ӷ���byte����
                                                                         @param cePackageSend:CePackageSend���Զ���ָ��
                                                                         @return ������ֱ�ӷ��͵�byte���飬����ΪCE_PACKAGE_SEND_BUF_SIZE*/

    CE_STATUS   (*dealRecv)(CePackageRecv* cePackageRecv, uint8* recvBuf,uint16 recvCount); /*!< 
                                                                         @brief ��recvBuf�е����ݽ��в����������������Ľ�����µ��ṹ��cePackageRecv
                                                                         @param cePackageRecv:CePackageRecv���Զ���ָ��
                                                                         @param recvBuf:�������ݻ����ַ
                                                                         @param recvCount:�������ݻ��泤��
                                                                         @return ����CE_STATUS_SUCCESS������ɹ�������ʧ��*/

}CePackageOp;
/*
 *CePackage��������ʵ��
 */
extern const CePackageOp cePackageOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_PACKAGEH__

