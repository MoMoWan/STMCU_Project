/**
  ******************************************************************************
  * @file    main.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-23
  * @brief   CREELINKSС�������˻�����������ڳ���
  ******************************************************************************
  * @attention
  *
  * 1)ע�⣺������ֱ���Ķ�Դ���룬���ȼ�Ҫ����CREELINKSС�������˻�������ṹ����ܣ�
  *   ���ص�ַ��http://www.creelinks.com/uav
  * 2)ע�⣺�й�CREELINKS����ӿڵĶ��塢ʹ�÷�������ֲ����������ʹ�������������ϡ�
  *   �ٷ���վ��http://www.creelinks.com/stdlib
  * 3)ע�⣺�ɼ���CREELINKSС���ύ��Ⱥ623083844
  *
  * 1)����CREELINKSƽ̨V1.0
  * 2)֧��WIFI��������2.4G��Ƶ����ͨѶ��ʽ����ǰ�汾(V1.0)�ݲ�֧������ͨѶ
  * 3)����ʱ�����Ұ�ť��ֱ��������..��..������ϵͳ��������������ʽ
  * 4)����ʱ������ť��ֱ��������..һ����ϵͳ����WIFI������ʽ
  * 5)����ʱ�����κΰ�ť��ϵͳĬ�Ͻ���2.4G��Ƶ������ʽ
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "Creelinks.h"                  /*!< CREELINKSƽ̨ͷ�ļ�*/
#include "Ce6Dof.h"                     /*!< MPU6050 6�ᴫ����ģ������*/
#include "CeBmp180.h"                   /*!< BMP180��ѹ������ģ������*/
#include "CePID.h"                      /*!< PID��������*/
#include "CeFilter.h"                   /*!< �������˲�����Ԫ������̬����*/
#include "CeTMU.h"                      /*!< ���ݴ������ģ������*/
#include "CeMD.h"                       /*!< ����������ģ������*/
#include "CeLedCtl.h"                   /*!< �ĸ�LED�ƿ��ƣ�ͨ����˸��ϣ��������˻���ǰ�Ĺ���״̬*/
#include "CeBtnx1.h"                    /*!< ���˻��ϣ�����������ťģ������*/
#include "CePC33V.h"                    /*!< ��ص�ѹ���ģ������������ʵ�ֵ�ص������*/
#include "CeBeepNSrc.h"                 /*!< ���������*/
#include "CeFilter.h"                   /*!< �˲������*/
#include "CePackage.h"                  /*!< ���ݴ��������*/

#define     UAV_STATUS_READY        0   /*!< ��ǰ���˻�����״̬���������ȴ���ɣ���ʱ�ĸ��������ֹͣ����״̬*/
#define     UAV_STATUS_FREEDOM_FLY  1   /*!< ��ǰ���˻�����״̬�����ɷ���״̬�����ң�˿������˻�����*/
#define     UAV_STATUS_FIX_HIGHT_FLY 2  /*!< ��ǰ���˻�����״̬�����߷���״̬�����ң�˿������˻����θ߶�*/
  
Ce6Dof      ce6Dof;                     /*!< ����6�ᴫ��������*/
CeBmp180    ceBmp180;                   /*!< ����BMP180����ѹ������������*/
CeMD        ceMD0;                      /*!< �����0·���������ƶ���*/
CeMD        ceMD1;                      /*!< �����1·���������ƶ���*/
CeMD        ceMD2;                      /*!< �����2·���������ƶ���*/
CeMD        ceMD3;                      /*!< �����3·���������ƶ���*/
CeBtnx1     ceBtnLeft;                  /*!< �������˻���ť����*/
CeBtnx1     ceBtnRight;                 /*!< �������˻��Ұ�ť����*/
CePC33V     cePC33V;                    /*!< �����ص�ѹ������*/
CeBeepNSrc  ceBeepNSrc;                 /*!< ������Դ����������*/

CeAcc       ceNowAcc;                   /*!< ���嵱ǰ���˻����ٶȽṹ��*/
CeGyr       ceNowGyr;                   /*!< ���嵱ǰ���˻����ٶȽṹ��*/
CeAngles    ceNowAngles;                /*!< ���嵱ǰ�����˻���̬*/  
CeAngles    ceHopeAngles;               /*!< �����û����������˻���̬*/    
  
Ce6DofAcceleration*    acc;             /*!< 6�ᴫ�������ٶ�����*/
Ce6DofGyroscope*       gyr;             /*!< 6�ᴫ�������ٶ�����*/
CeDrivePower*          ceDrivePower;    /*!< �����ĸ����������ǿ�Ƚṹ��ָ��*/
CeBmp180Environment*   ceEnvirmont;     /*!< ����BMP180�ɼ����ĺ����¶ȵ�ָ������*/
CePackageSend       cePackageSend;      /*!< ���ݴ�������͹������*/
CePackageRecv       cePackageRecv;      /*!< ���ݽ��ղ�����������*/

uint8   uavStatus = UAV_STATUS_READY;   /*!< ���˻�����״ָ̬ʾ����*/
fp32    dtS = 0;                        /*!< ������whileִ�����ڣ���λS*/

/**
  * @brief  �������˻�״̬��Ϣ�����ƶ�
  * @return ��
  */
void sendStatusToCtl(void) 
{
    cePackageSend.batVoltage = (int32)(cePC33VOp.getVoltage(&cePC33V)*1000);             //���ϵ�ǰ���˻���ص�ѹֵ������ͳ�Ƶ�ص���,cePackageSend�е�������������CePID��CeFilter�и���
    cePackageSend.pressure = (int32)(ceEnvirmont->pressure);
    cePackageSend.temperature = (int32)(ceEnvirmont->temperature*1000);
    cePackageSend.altitude = (int32)(ceEnvirmont->altitude*1000);
    cePackageSend.accelerator = (int32)(ceNowAngles.accelerator);
    if(ceTMUOp.getSendIntervalMs() >= 35)  //�������η������ݵ�ʱ����
        ceTMUOp.sendData(cePackageOp.dealSend(&cePackageSend,cePackageRecv.status),CE_PACKAGE_SEND_BUF_SIZE);    //��cePackageSend�ṹ�������˻�״̬���ݴ���������͸�����վ��ң����
}

/**
  * @brief  TMU���ݴ������ģ��������ݺ󣬵��õĻص�����
  * @param  dataBuf:���յ����ݵĻ����ַ
  * @param  dataCount:���յ����ݵĳ���
  * @return ��
  */
void recvDataCallBack(uint8* dataBuf, uint16 dataCount) 
{
    if(CE_STATUS_SUCCESS != cePackageOp.dealRecv(&cePackageRecv,dataBuf,dataCount))//���ݽ��㲻��ȷ����ֱ�ӷ���
        return;
    ceHopeAngles.yaw = ((ceMathOp.abs(cePackageRecv.leftX) > 700) ? (-(fp32)(cePackageRecv.leftX)/50):0.0f);        //����ƫ����������700����������ֹ������ʱƫ�������·��в��ȶ�
    ceHopeAngles.picth = ((ceMathOp.abs(cePackageRecv.rightY) > 150) ? ((fp32)(cePackageRecv.rightY) /166) : 0.0f);  //���ƽǶ���-6~+6��֮��,������Pitch�ǵ��������Է�ֹҡ�����µ�Ʈ��
    ceHopeAngles.roll = ((ceMathOp.abs(cePackageRecv.rightX) > 150) ? (-(fp32)(cePackageRecv.rightX) /166) : 0.0f);  //���ƽǶ���-6~+6��֮�� ,������Roll�ǵ��������Է�ֹҡ�����µ�Ʈ��       

    if(uavStatus == UAV_STATUS_FREEDOM_FLY)
        ceHopeAngles.accelerator = cePackageRecv.leftY/2 + 500;     //��-1000~+1000��ת��Ϊ0~1000���Ա�֤����������0~1000�ķ�Χ��        
    else if(uavStatus == UAV_STATUS_FIX_HIGHT_FLY)
        ceHopeAngles.altitude += (ceMathOp.abs(cePackageRecv.leftY) > 200)?((fp32)(cePackageRecv.leftY)/10000):0.0f;   //��������������ҡ��λ���м�ʱ�ľ�������߶����ӻ��С.

    if(ceHopeAngles.picth >3)           ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_FRONT);
    else if(ceHopeAngles.picth <(-3))   ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_BACK);
    else if(ceHopeAngles.roll >3)       ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_LEFT);
    else if(ceHopeAngles.roll <(-3))    ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_RIGHT);
    else if(ceHopeAngles.yaw >3)        ceLedCtlOp.setMode(CE_LED_CTL_MODE_FLASH_CYCLE_N);
    else if(ceHopeAngles.yaw <(-3))     ceLedCtlOp.setMode(CE_LED_CTL_MODE_FLASH_CYCLE_P);
    else                                ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);
    if((cePackageRecv.status & CE_CTL_BTN_S2D) != 0)//ң�������¼�ͣ��ťS2D���ر��ĸ������������������˻�����Ϊ׼��״̬
    {
        uavStatus = UAV_STATUS_READY;
        ceHopeAngles.accelerator = 0;
        ceHopeAngles.picth = 0;
        ceHopeAngles.roll = 0;
        ceHopeAngles.yaw = 0;
        ceHopeAngles.altitude = -9999;//����֪ͨPID������ʶ��ǰ���˻��Ƕ��߷��У��������ɷ���
    }
    else if(((cePackageRecv.status & CE_CTL_BTN_RIGHT) != 0) && (cePackageRecv.leftY <= (-800)))//����������ҡ���������(����С��200)��������ҡ�˰�ťʱ,���ҵ�ص�ѹ����2.6Vʱ����������״̬��
    {
        uavStatus = UAV_STATUS_FREEDOM_FLY;
        ceHopeAngles.altitude = -9999;//����֪ͨPID������ʶ��ǰ���˻��Ƕ��߷��У��������ɷ���
    }  
    else if((cePackageRecv.status & CE_CTL_BTN_RIGHT) != 0)//������S2Aʱ�����˻����ڶ��߷���״̬
    {
        uavStatus = UAV_STATUS_FIX_HIGHT_FLY;
        ceHopeAngles.altitude = ceNowAngles.altitude+ ((ceNowAngles.accelerator <= 200)?1.5f:0);//��ǰ����С��200��˵�����˻����ڵ����ϣ����ʱ�Ľ������߶�����Ϊ���ڵ���1.5�ף������Ŵ���200ʱ��˵�����˻����ڷ��У���ʱ��Ҫ�������߶�����Ϊ��ǰ�߶�
    }  
}

/**
  * @brief  ���ͨ�������Ƿ�����
  * @return ��
  */
void checkConnectStatus(void)
{
    if(ceTMUOp.checkConnectStatus() != CE_STATUS_SUCCESS)//��������˻�ʧ������̬���У�������BEEP������ʾ��
    {
        if(uavStatus == UAV_STATUS_FREEDOM_FLY)
            ceHopeAngles.accelerator -= (ceHopeAngles.accelerator>10? 10:0);
        else if(uavStatus == UAV_STATUS_FIX_HIGHT_FLY)
            ceHopeAngles.altitude -=  0.1f;        

        ceHopeAngles.picth = 0;
        ceHopeAngles.roll = 0;
        ceHopeAngles.yaw = 0;
        ceBeepNSrcOp.say(&ceBeepNSrc,30,0,1);

        if(ceNowAngles.accelerator <= CE_PID_MIN_DRIVER_POWER + 400) uavStatus = UAV_STATUS_READY;//������С��һ��ֵ�󣬹رյ��������׼��״̬
        ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_CFG);//LED��״̬Ϊ����״̬

    }else if(ceLedCtlOp.getMode() == CE_LED_CTL_MODE_IN_CFG)
        ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);//LED��״̬Ϊ����״̬
}

/**
  * @brief  ������˻��Ƿ�����������������������ʱ��1s����رյ���������������׼��״̬
  * @return ��
  */
void checkTurnOver(void)
{
    static fp32 turnTime = 0;
    if((uavStatus != UAV_STATUS_READY ) && ((cePackageRecv.status & CE_CTL_TYPE_STATION) == 0))//���ڷ���ģʽ�����ҷǵ���վ����ģʽ�£��Ž��з������
    {
        if(ceNowAcc.z < (-0.8f)) turnTime += dtS;
        else turnTime = 0;
        if(turnTime >= 1.0f)
        {
            ceHopeAngles.picth = 0;
            ceHopeAngles.roll = 0;
            ceHopeAngles.yaw = 0;
            ceHopeAngles.accelerator = 0;
            uavStatus = UAV_STATUS_READY;
        }
    }
}

/**
  * @brief  ���������whileѭ����ִ������
  * @return whileִ�����ڣ���λs
  */
void calSystemRunCycle(void)
{
    static uint64  lastRecordSysTimeUs= 0;
    dtS = (fp32)(ceSystemOp.getSystemTickUs() - lastRecordSysTimeUs)/1000000;
    lastRecordSysTimeUs = ceSystemOp.getSystemTickUs();
    if(dtS > 0.030f) dtS = 0.030f;//�����������Ϊ30ms
}

/**
  * @brief  ��ʼ�����Զ���ṹ������
  * @return ��
  */
void initialParment(void)              
{
    ceHopeAngles.accelerator = 0;
    ceHopeAngles.roll = 0;
    ceHopeAngles.picth = 0;
    ceHopeAngles.yaw = 0;
    
    ceHopeAngles.altitude = -9999;
}

/**
  * @brief  ��ʼ��������ģ��
  * @return ��
*/
void initialModule(void)
{   
    ceMDOp.initial(&(ceMD0), PC6GIP);                                   //ʹ��һ·Pwm��Դ�ų�ʼ��������0
    ceMDOp.initial(&(ceMD0), PC6GIP);                                   //ʹ��һ·Pwm��Դ�ų�ʼ��������0,ע���˴���ʼ������PC6��Ӧ��Pwm������ΪSTM32F103��TIM8��ʱ���ƺ���BUG����·ͨ������ʼ��������˳���Ӱ�������PWM�����򣬺�������о���
    ceMDOp.initial(&(ceMD1), PC7GIP);                                   //ʹ��һ·Pwm��Դ�ų�ʼ��������1
    ceMDOp.initial(&(ceMD3), PC9GIP);                                   //ʹ��һ·Pwm��Դ�ų�ʼ��������3
    ceMDOp.initial(&(ceMD2), PC8GIP);                                   //ʹ��һ·Pwm��Դ�ų�ʼ��������2

    ceBeepNSrcOp.initialByGpio(&ceBeepNSrc,PD2CGI);                     //ʹ��һ·Gpio��Դ�ڳ�ʼ����Դ������

    ceLedCtlOp.initial(PC0AGI,PC1AGI,PC2AGI,PC3AGI);                    //ʹ��4��Gpio��Դ�Ž���������ʾ���˻�����״̬���ĸ�LED��ʼ��
    ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_CFG);                         //LED����ʾΪ����״̬

    ceBtnx1Op.initialByGpio(&ceBtnLeft,PA11GIP,CE_NULL);                //ʹ��Gpio�ڳ�ʼ����ťģ��
    ceBtnx1Op.initialByGpio(&ceBtnRight,PA12CGI,CE_NULL);               //ʹ��Gpio�ڳ�ʼ���Ұ�ťģ��

    
    if(ceBtnx1Op.getStatus(&(ceBtnLeft)) == 0x01)                       //�򿪿���ʱ��������ť�������WIFIͨѶģʽ
    {
        ceBeepNSrcOp.say(&ceBeepNSrc,100,0,1);   
        ceTMUOp.initial(CE_TMU_USE_WIFI,recvDataCallBack);              //��ʼ�����ݴ������,ÿ20ms����һ�����ݷ��͡�ע��ֱ����������ƶ˽���ͨѶ�����ŷ���
    }
    else if(ceBtnx1Op.getStatus(&(ceBtnRight)) == 0x01)                 //�򿪿���ʱ�������Ұ�ť�����������ͨѶģʽ
    {
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,2);
        ceTMUOp.initial(CE_TMU_USE_BLUE,recvDataCallBack);      
    }
    else                                                                //������������NRF24L01ͨѶ��ʽ            
    {
        ceTMUOp.initial(CE_TMU_USE_NRF,recvDataCallBack); 
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,3);            
        ceSystemOp.delayMs(3000);                                       //��ʱһ���ʱ�䣬��ȷ�����˻����ڵ��澲ֹ״̬����ֹMPU6050��ʼ��������ٶ����ʱ�������
    }

    ce6DofOp.initial(&ce6Dof,I2c1);                                     //ʹ��I2c��Դ�ų�ʼ��6�ᴫ����
    ceBmp180Op.initial(&ceBmp180,I2c1);                                 //ʹ��I2c��Դ�ų�ʼ��BMP180����ѹ��������

    cePackageOp.initialRecv(&cePackageRecv);                            //��ʼ�����ݰ��������
    cePackageOp.initialSend(&cePackageSend);                            //��ʼ�����ݰ��������

    ceFilterOp.initial(&cePackageSend,&cePackageRecv);                  //���˻���̬�������˲����ƶ������֣�һ�����ס���Ԫ����������
    cePIDOp.initial(&cePackageSend,&cePackageRecv);                     //���˻�����PID���ƶ���

    cePC33VOp.initial(&cePC33V,PC4AGI);                                 //ʹ��һ·Ad��Դ�ڳ�ʼ����ѹ����ģ��

    ceBeepNSrcOp.say(&ceBeepNSrc,1000,0,1);                             //����������1S�죬��ʾ�ѳ�ʼ����ɣ��������
    ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);                      //LED����ʾΪ�������״̬
}

/**
  * @brief  CREELINKSƽ̨����ں���(�㱼)
  * @return 0
  */
int main(void)
{
    ceSystemOp.initial();                           //Creelinks������ʼ��
    ceDebugOp.initial(Uart4);                       //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    initialParment();                               //�ṹ��Ĳ�����ʼ��
    initialModule();                                //��ʼ�����й���ģ��
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���        
        //TODO:���ڴ˴������û�����   

        acc = ce6DofOp.getAcceleration(&ce6Dof);    //��ȡ��ǰ���˻����ٶ�
        gyr = ce6DofOp.getGyroscope(&ce6Dof);       //��ȡ��ǰ���˻����ٶ�        
        ceEnvirmont = ceBmp180Op.getEnvironmentAsync(&ceBmp180);//��ȡ��ǰ���˻�����ѹ�������

        ceNowAcc.x = acc->x ;                       //ת����ٶ�����
        ceNowAcc.y = acc->y ;  
        ceNowAcc.z = acc->z ; 

        ceNowGyr.x = -gyr->x;                       //ת����ٶ�����
        ceNowGyr.y = gyr->y;
        ceNowGyr.z = gyr->z;

        ceNowAngles.altitude = ceEnvirmont->altitude;//��ǰ�߶��ݴ�

        ceFilterOp.filter(&ceNowAcc,&ceNowGyr,&ceNowAngles,dtS);            //�Ե�ǰ���ٶȡ���ǰ���ٶȽ�����̬�������˲����Ի�ȡ���˻���̬�����ݣ�ע���Ὣ���������µ�����ָ�������ָ���ݡ�            
        ceDrivePower = cePIDOp.calculate(&ceNowAcc, &ceNowGyr, &ceNowAngles, &ceHopeAngles,dtS);   //���ݵ�ǰ���ٶȼ����ٶȡ���̬�ǡ�����������̬���д���PID���㣬������ĸ����������ǿ��                

        ceMDOp.setDriverPower(&ceMD0,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower0):0);    //���õ�0·�������ǿ�ȣ�0~1000                
        ceMDOp.setDriverPower(&ceMD1,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower1):0);    //���õ�1·�������ǿ�ȣ�0~1000
        ceMDOp.setDriverPower(&ceMD2,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower2):0);    //���õ�2·�������ǿ�ȣ�0~1000
        ceMDOp.setDriverPower(&ceMD3,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower3):0);    //���õ�3·�������ǿ�ȣ�0~1000

        sendStatusToCtl();                          //���ϵ�ǰ���ݣ������͸����ƶ�
        checkConnectStatus();                       //��������Ƿ�Ͽ�������Ͽ������˻���̬���в������½�
        calSystemRunCycle();                        //�������ִ������ʱ��
        checkTurnOver();                            //������˻��Ƿ񷭻����������رյ��������Ա���MOS�ܼ����
    };
}
