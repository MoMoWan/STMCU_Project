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
  * 3)����ʱ����S2C��ֱ��������..��..������ϵͳ��������������ʽ
  * 4)����ʱ����S2D��ֱ��������..һ����ϵͳ����WIFI������ʽ
  * 5)����ʱ�����κΰ�ť��ϵͳĬ�Ͻ���2.4G��Ƶ������ʽ
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "Creelinks.h"          /*!< CREELINKSƽ̨ͷ�ļ�*/
#include "CeBtnx1.h"            /*!< ���˻��ϣ�����������ťģ������*/
#include "CePC33V.h"            /*!< ��ص�ѹ���ģ������������ʵ�ֵ�ص������*/
#include "CeBeepNSrc.h"         /*!< ���������*/
#include "CePackage.h"          /*!< ���ݴ��������*/
#include "CeJoystick.h"         /*!< ҡ�����*/
#include "CeTft180.h"           /*!< TFT��ʾ�����*/
#include "CeTMU.h"              /*!< ͨѶ�������*/
#include "CeLed1C.h"            /*!< LED�����*/

#define CE_UAV_CTL_TEST     0   /*!< ���õ�ǰң�������ڲ���״̬*/
#define CE_UAV_CTL_NORMAL   1   /*!< ���õ�ǰң����������������״̬*/

CeBeepNSrc  ceBeepNSrc;         /*!< ������Դ����������*/
CePC33V     cePC33V;            /*!< �����ص�ѹ������*/
CeJoystick  ceJoystickLeft;     /*!< �������ҡ�˶���*/
CeJoystick  ceJoystickRight;    /*!< �����Ҳ�ҡ�˶���*/
CeTft180    ceTft180;           /*!< ����TFT180��ʾ������*/

CeBtnx1     ceBtnS2A;           /*!< ����S2A��ť����*/
CeBtnx1     ceBtnS2B;           /*!< ����S2B��ť����*/
CeBtnx1     ceBtnS2C;           /*!< ����S2C��ť����*/
CeBtnx1     ceBtnS2D;           /*!< ����S2D��ť����*/

CeLed1C     ceLed1A;            /*!< ����LED��A����*/
CeLed1C     ceLed1B;            /*!< ����LED��B����*/
CeLed1C     ceLed1C;            /*!< ����LED��C����*/
CeLed1C     ceLed1D;            /*!< ����LED��D����*/

extern CeTMU    ceTMU;          /*!< ��������ͨѶ�������*/

CePackageSend cePackageSend;    /*!< �������ݴ�������Ͷ���*/
CePackageRecv cePackageRecv;    /*!< �������ݲ������������*/

uint8   workModel = CE_UAV_CTL_NORMAL;   /*!< ָʾ��ǰң�����Ĺ���״̬*/
/**
  * @brief  ����ң����״̬�����˻�
  * @return ��
  */
void sendStatusToCtl()
{
    CeJoystickAxis* axis;

    ceLed1COp.setOn(&ceLed1D);                          //�򿪷�����ʾLED��LED1D
    axis = ceJoystickOp.getAxis(&ceJoystickLeft);       //��ȡ��ҡ�˵�λ�ã�X�ᣭ1000~+1000��Y��-1000~+1000
    cePackageSend.leftX = axis->y;                      //��ʹ����CREELINKSҡ�˵�����ģ�鼰�����⣬��ģ��İ�װ������CREELINKS�����еķ����в��죬�������X�ἰY�����
    cePackageSend.leftY = -axis->x;
    axis = ceJoystickOp.getAxis(&ceJoystickRight);
    cePackageSend.rightX = -axis->y;
    cePackageSend.rightY = axis->x;
    cePackageSend.status =                              //����6����ť�����ݣ�bit0:leftBtn, bit1:rightBtn, bit2:S2A, bit3:S2B, bit4:S2C, bit5:S2D
        ((ceJoystickOp.getBtnStatus(&ceJoystickLeft)== 0x00 )? 0x0000:CE_CTL_BTN_LEFT) | ((ceJoystickOp.getBtnStatus(&ceJoystickRight)== 0x00 )? 0x0000:CE_CTL_BTN_RIGHT)| 
        ((ceBtnx1Op.getStatus(&ceBtnS2A)== 0x00 )? 0x0000:CE_CTL_BTN_S2A)|((ceBtnx1Op.getStatus(&ceBtnS2B) == 0x00 )? 0x0000:CE_CTL_BTN_S2B)| 
        ((ceBtnx1Op.getStatus(&ceBtnS2C) == 0x00 )? 0x0000:CE_CTL_BTN_S2C)|((ceBtnx1Op.getStatus(&ceBtnS2D) == 0x00 )? 0x0000:CE_CTL_BTN_S2D) | CE_FILTER_IN_KALMAN | CE_CTL_TYPE_CTL;
    //����PID�Ȳ���Ĭ��Ϊ0������ʱ�����͵�һ��������PID�������÷��͸����˻�
    ceTMUOp.sendData(cePackageOp.dealSend(&cePackageSend),CE_PACKAGE_PACK_SIZE);//ֻ��һ֡��ȥ����
    ceLed1COp.setOff(&ceLed1D);                         //�رշ��Ϳ���LED��LED1D
}

/**
  * @brief   ����ǰ���˻���̬���ݸ��µ�TFT���ϣ�ע��SPIͨѶ��ʽ�ٶȲ��ߣ��˺����Ứ�ѹ���ʱ��
  * @return ��
  */
void showStatus()
{
    char temp[32];
    if(workModel == CE_UAV_CTL_TEST)
    {
        CeJoystickAxis* axis;
        uint16 showIndex = 0;
        axis = ceJoystickOp.getAxis(&ceJoystickLeft);   //��ȡ��ҡ�˵�λ�ã�X�ᣭ1000~+1000��Y��-1000~+1000
        ceDebugOp.sprintf(temp,"LeftX:%d        ",axis->y);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"LeftY:%d        ",-axis->x);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        axis = ceJoystickOp.getAxis(&ceJoystickRight);
        ceDebugOp.sprintf(temp,"RightX:%d        ",-axis->y);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"RightY:%d        ",axis->x);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        
        showIndex++;
        ceDebugOp.sprintf(temp,"LeftBtn:%s        ",((ceJoystickOp.getBtnStatus(&ceJoystickLeft) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"RightBtn:%s        ",(ceJoystickOp.getBtnStatus(&ceJoystickRight) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        
        showIndex++;
        ceDebugOp.sprintf(temp,"S2A:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2A) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2B:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2B) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2C:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2C) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2D:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2D) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
    }else
    {
        uint8 static showTime = 0;//TFT180ˢ����ֻ��10HZ��Ϊ���ⳤʱ�����mainTask()������������÷ֶϸ��·�ʽ��ȷ��mainTask()�����������С��35ms

        ceDebugOp.sprintf(temp,"Send:%d        ",ceTMU.sendPackCount);
        ceTft180Op.showString(&ceTft180,0,0*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"recv:%d        ",ceTMU.recvPackCount);
        ceTft180Op.showString(&ceTft180,0,1*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

        if(showTime == 0)
        {
            showTime++;
            ceDebugOp.sprintf(temp,"acc:%f        ",(fp32)(cePackageRecv.accelerator));
            ceTft180Op.showString(&ceTft180,0,3*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

            ceDebugOp.sprintf(temp,"Pitch:%f        ",(fp32)(cePackageRecv.pitchByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,5*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
            ceDebugOp.sprintf(temp,"Roll:%f        ",(fp32)(cePackageRecv.rollByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,6*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

        }else if(showTime == 1)
        {
            showTime++;
            ceDebugOp.sprintf(temp,"Yaw:%f        ",(fp32)(cePackageRecv.yawByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,7*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

            ceDebugOp.sprintf(temp,"temp:%f        ",(fp32)(cePackageRecv.temperature)/1000);
            ceTft180Op.showString(&ceTft180,0,9*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
            ceDebugOp.sprintf(temp,"alt:%f        ",(fp32)(cePackageRecv.altitude)/1000);
            ceTft180Op.showString(&ceTft180,0,10*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        }else  if(showTime == 2)
        {
            showTime = 0;

            ceDebugOp.sprintf(temp,"Vol:%f        ",(fp32)(cePackageRecv.batVoltage)/1000);
            ceTft180Op.showString(&ceTft180,0,12*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        }
    }

}

/**
  * @brief  TMU���ݴ������ģ��������ݺ󣬵��õĻص�����
  * @param  dataBuf:���յ����ݵĻ����ַ
  * @param  dataCount:���յ����ݵĳ���
  * @return ��
  */
void recvDataCallBack(uint8* dataBuf, uint16 dataCount)
{
    ceLed1COp.setOn(&ceLed1C);                                  //�򿪽��մ���LEDָʾ��LED1C
    cePackageOp.dealRecv(&cePackageRecv,dataBuf,dataCount);     //�Խ������ݽ��в������
    ceLed1COp.setOff(&ceLed1C);                                 //�رս��մ���LEDָʾ��LED1C
}

/**
  * @brief  ��CREELINKSƽ̨ϵͳ���ã�����ʵ������CMD���ڵĹ���
  * @return ��
  */
void appendString(const char* msg)
{
    ceTft180Op.appendString(&ceTft180,msg);
}

/**
  * @brief  S2C��ť�Ļص����������Կ���TFT180��ʾ���Ŀ���
  * @return ��
  */
void btnS2cCallBack()
{
    if(ceTft180Op.getShowStatus(&ceTft180) == 0x01)
        ceTft180Op.setOff(&ceTft180);   //�ر�TFT180��ʾ
    else 
        ceTft180Op.setOn(&ceTft180);    //����TFT180��ʾ
}

/**
  * @brief  ��ʼ��������ģ��
  * @return ��
  */
void initialModule(void)
{
    ceTft180Op.initial(&ceTft180,Spi2,PB9GI,PB10GIP,PB11GIP);       //ʹ��һ·Spi����·GPIO��Դ��ʼ��TFT180��ʾģ��

    ceLed1COp.initialByGpio(&ceLed1A,PC10GI);                       //ʹ��һ·GPIO��Դ����ʼLED1A
    ceLed1COp.initialByGpio(&ceLed1B,PC11GI);                       //ʹ��һ·GPIO��Դ����ʼLED1B
    ceLed1COp.initialByGpio(&ceLed1C,PC12GI);                       //ʹ��һ·GPIO��Դ����ʼLED1C
    ceLed1COp.initialByGpio(&ceLed1D,PC13GI);                       //ʹ��һ·GPIO��Դ����ʼLED1D

    ceTft180Op.showString(&ceTft180,28,56,0x0b0f,CE_TFT180_COLOR_BLACK,"CREELINKS",CE_TFT180_EN_SIZE_F8X16);//��ʾ2s CREELINKSƽ̨�Ĵ�LOGO
    ceSystemOp.delayMs(2000); 

    ceDebugOp.registerAppendString(appendString);                   //��CREELINKSϵͳע��һ������ʵ��CMD��ʾ���ܵĺ���

    ceDebugOp.printf("Initial LED...\n");    
    ceLed1COp.setFlash(&ceLed1A,80,500);                            //����LED1A��Ϊ��˸״̬

    ceDebugOp.printf("Initial Beep...\n");
    ceBeepNSrcOp.initialByGpio(&ceBeepNSrc,PA1AGIP);                //ʹ��һ·Gpio��Դ�ڳ�ʼ����Դ������

    ceDebugOp.printf("Initial PC33V...\n");    
    cePC33VOp.initial(&cePC33V,PA0ACGIP);                           //ʹ��һ·Ad��Դ�ڣ���ʼ��PC33V��ѹ����ģ��

    ceDebugOp.printf("Initial Joystick...\n");
    ceJoystickOp.initial(&ceJoystickLeft,PC0AGI,PC1AGI,PC2AGI);     //ʹ����·Ad��һ·Gpio��ʼ�����ҡ��ģ��
    ceJoystickOp.initial(&ceJoystickRight,PC3AGI,PC4AGI,PC5AGI);    //ʹ����·Ad��һ·Gpio��ʼ���Ҳ�ҡ��ģ��

    ceDebugOp.printf("Initial Btn...\n");
    ceBtnx1Op.initialByGpio(&ceBtnS2A,PC6GIP,CE_NULL);              //ʹ��һ·Gpio�ڳ�ʼ��������ťģ��S2A������ע��ص�����
    ceBtnx1Op.initialByGpio(&ceBtnS2B,PC7GIP,CE_NULL);              //ʹ��һ·Gpio�ڳ�ʼ��������ťģ��S2B������ע��ص�����
    ceBtnx1Op.initialByGpio(&ceBtnS2C,PC8GIP,btnS2cCallBack);       //ʹ��һ·Gpio�ڳ�ʼ��������ťģ��S2C��ע��ص������Կ���TFT������ʾ
    ceBtnx1Op.initialByGpio(&ceBtnS2D,PC9GIP,CE_NULL);              //ʹ��һ·Gpio�ڳ�ʼ��������ťģ��S2D������ע��ص�����

    if(ceBtnx1Op.getStatus(&ceBtnS2D) == 0x01)                      //ϵͳ�ϵ�ʱ������S2D��ť�������WIFIͨѶģʽ
    {
        ceDebugOp.printf("Initial TMU By Wifi...\n");

        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,1);                    //��1��
        ceTMUOp.initialByWifi(recvDataCallBack);                    //��TMU���ݴ���ģ����г�ʼ������ʱ�ϳ�
    }
    else if(ceBtnx1Op.getStatus(&ceBtnS2C) == 0x01)                 //ϵͳ�ϵ�ʱ������S2C��ť�����������ͨѶģʽ
    {
        ceDebugOp.printf("Initial TMU By BLUE...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,2);                    //��2��
        ceTMUOp.initialByBlue(recvDataCallBack);      
    }else if(ceBtnx1Op.getStatus(&ceBtnS2A) == 0x01)                //�����������״̬
    {
        ceDebugOp.printf("IN TEST WORK TYPE...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,5);                    //��5��
        workModel = CE_UAV_CTL_TEST;                                //�������״̬
    }else                                                           //������������NRF24L01ͨѶ��ʽ            
    {
        ceDebugOp.printf("Initial TMU By NRF24L01+...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,3);
        ceTMUOp.initialByNrf(recvDataCallBack,sendStatusToCtl);               
    }

    ceDebugOp.printf("Initial Package...\n");        
    cePackageOp.initialSend(&cePackageSend);                        //�����ݴ������ģ����г�ʼ��
    cePackageOp.initialRecv(&cePackageRecv);                        //�����ݽ��ղ��ģ����г�ʼ��

    ceLed1COp.setFlash(&ceLed1A,30,1000);                           //����LED1A��Ϊ��������״̬

    ceDebugOp.printf("System Initial Finish...\n");    
    ceBeepNSrcOp.say(&ceBeepNSrc,1000,0,1);                         //��������1s����ʾϵͳ��ʼ�����

    ceDebugOp.unRegisterAppendString();                             //ȡ��ע��ϵͳ��CMD���ܺ������˺������ʹ��TFT��ʾ��
    ceTft180Op.fill(&ceTft180,CE_TFT180_COLOR_BLACK);               //����ΪBLACK
}

/**
  * @brief  CREELINKSƽ̨����ں���(�㱼)
  * @return 0
  */
int main()
{
    ceSystemOp.initial();                                           //Creelinks������ʼ��
    ceDebugOp.initial(Uart4);                            //ͨ��Uart�������Debug��Ϣ����λ��
    initialModule();                                                //��ʼ��������ģ��
    while (1)
    {
        ceTaskOp.mainTask();                                        //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        showStatus();                                               //�����˻�״̬��Ϣ��ʾ��TFT����
    }
}



