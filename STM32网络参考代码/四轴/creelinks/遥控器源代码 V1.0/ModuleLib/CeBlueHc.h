/**
  ******************************************************************************
  * @file    CeBlueHc.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBlueHcģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BLUE_HC_H__
#define __CE_BLUE_HC_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BLUE_HC_VERSION__ 1                                             /*!< �������ļ��İ汾��*/
#define __CE_BLUE_HC_NEED_CREELINKS_VERSION__ 1                              /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_BLUE_HC_NEED_CREELINKS_VERSION__)       /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeBlueHc.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

#define CE_BLUE_HC_RECV_BUF_SIZE        256                            /*!< �������н��յĻ��泤��*/
#define CE_BLUE_HC_DEV_LIST_SIZE        3                               /*!< ָʾ�����豸ʱ�����ɲ��ҵ��Ĵ��豸����*/
#define CE_BLUE_HC_FIND_DEV_OUT_TIME    48                              /*!< ָʾ�����豸�ĳ�ʱʱ��*/


#define  CE_BLUE_HC_DEV_TYPE_NO_STANDARD    "0x1F1F"                           /*!< �Ǳ�׼�����豸��*/
#define  CE_BLUE_HC_DEV_TYPE_IPHONE         "0x7A020C"                          /*!< ƻ���ֻ������豸��*/
#define  CE_BLUE_HC_DEV_TYPE_ANDROID        "0x5A020C"                          /*!< ��׿�ֻ������豸�ࣨ���壬��Ϊ�ȣ�*/


/**
  * @brief  ö�٣�CeBlueHc����Ĺ���ģʽ
  */
typedef enum
{
    CE_BLUE_HC_WORK_MODE_MASTER = 1,                                    /*!< ���豸�����������������豸��������������*/
    CE_BLUE_HC_WORK_MODE_SLAVE = 0,                                     /*!< ���豸�����������豸���ң������������豸����������*/
    CE_BLUE_HC_WORK_MODE_LOOP = 2,                                      /*!< �ػ���ɫ���������ӣ�����Զ���������豸���ݲ�������ԭ�����ظ�Զ���������豸*/
}CE_BLUE_HC_WORK_MODE;


typedef struct
{
    char devName[32];                                                   /*!< ģ��ʹ�õ�Uart��Դ*/
    char devAdress[32];                                                 /*!< ģ��ʹ�õ�Uart��Դ*/
}CeBlueHcDevInfo;

/*
 *CeBlueHc���Զ���
 */
typedef struct
{
    CeUart  ceUart;                                                     /*!< ģ��ʹ�õ�Uart��Դ*/
    CeGpio  ceGpio0;                                                    /*!< ģ��ʹ�õ�Gpio��Դ*/
    CeGpio  ceGpio1;                                                    /*!< ģ��ʹ�õ�Gpio��Դ*/
    CeGpio  ceGpio2;                                                    /*!< ģ��ʹ�õ�Gpio��Դ*/
    CeBlueHcDevInfo ceBlueHcDevInfoList[CE_BLUE_HC_DEV_LIST_SIZE];      /*!< ������ģʽ�£����Ҵ��豸ʱ�����ҵ����豸����Ĵ�ŵ�*/
    uint8   ceBlueHcDevInfoFindDevCount;                                /*!< ��Ų��ҵ��Ĵ��豸����*/
    uint8   uartRecvBuf[CE_BLUE_HC_RECV_BUF_SIZE];                      /*!< Uart����ʹ�õ��Ľ��ջ���*/
    uint8   isLockRecvBuf;                                              /*!< �������ջ��棬��ֹ����ʱ�����ʱ���������ͻ*/
}CeBlueHc;
/*
 *CeBlueHc��������
 */
typedef struct
{
    CE_STATUS           (*initial)(CeBlueHc* ceBlueHc, CE_RESOURCE ceUart, CE_RESOURCE ceGpio0,CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2);/*!<
                                                                             @brief CeBlueHcģ���ʼ��
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param ceUart:CeBlueHcģ��ʹ�õ�Uart��Դ��
                                                                             @param ceGpio0-2:CeBlueHcģ��ʹ�õ�Gpio��Դ��*/

    CE_STATUS           (*parmentConfig)(CeBlueHc* ceBlueHc, CE_BLUE_HC_WORK_MODE ceBlueHcWorkMode, const char* ceBlueHcDevType,const char* devName, const char* password);/*!<
                                                                             @brief CeBlueHcģ���������
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param ceBlueHcWorkMode:CeBlueHcģ��Ĺ�����ʽ������ģ��ʹ�ģ��
                                                                             @param ceBlueHcDevType:��ģ�鹤������ģʽʱ�����Ҵ��豸ʱֻ���Ҵ����͵ģ���ģ�鹤���ڴ�ģʽʱ��Ϊ��ģ�������
                                                                             @param devName:���豸������
                                                                             @param password:���豸���������*/

    void                (*outParmentConfig)(CeBlueHc* ceBlueHc);        /*!< @brief �˳���������״̬��AT״̬����ģ�������ϵ磬��������������ģʽ
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��*/


    CeBlueHcDevInfo*    (*getCanConnectDevInfo)(CeBlueHc* ceBlueHc);    /*!< @brief ģʽ��������ģʽʱ��������Χ�п����ӵ�������Ϣ
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @return �������ӵ�������Ϣ����*/

    uint8               (*getCanConnectDevCount)(CeBlueHc* ceBlueHc);   /*!< @brief ģʽ��������ģʽʱ��������Χ�п����ӵ������豸����
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @return ���ؿ����ӵ������豸����*/

    CE_STATUS           (*checkDevIsExist)(CeBlueHc* ceBlueHc, const char* devBlueName);/*!<
                                                                             @brief ģʽ��������ģʽʱ������ָ���������Ƶ��豸�Ƿ���ڲ����ڿ�����״̬
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param devBlueName:��Ҫ���Ĵ��豸����
                                                                             @return ����CE_STATUS_SUCCESS��ʾ�����ӣ� ����������ʾ��������*/

    CE_STATUS           (*connectDevByName)(CeBlueHc* ceBlueHc, const char* devBlueName);/*!<
                                                                             @brief ģʽ��������ģʽʱ��ʹ���豸����������һ�����豸
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param devBlueName:��Ҫ���ӵĴ��豸����
                                                                             @return ����CE_STATUS_SUCCESS��ʾ���ӳɹ��� ����������ʾ����ʧ��*/

    uint8               (*getConnectStatus)(CeBlueHc* ceBlueHc);        /*!< @brief ��ȡģ�������״̬
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @return ����0x00:ģ�鴦��δ���״̬������0x01:ģ�鴦����Գɹ�״̬�����Խ������ݴ���*/

    void               (*sendData)(CeBlueHc* ceBlueHc, uint8* dataInBuf, uint16 sendCount);/*!<
                                                                             @brief ��������
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param dataInBuf:��Ҫ���͵����ݻ���
                                                                             @param sendCount:��Ҫ���͵����ݳ���
                                                                             @return ʵ�ʷ�����ɵ����ݳ���*/

    uint16              (*getRecvDataCount)(CeBlueHc* ceBlueHc);        /*!< @brief ��ȡ���ջ����еĿɶ�ȡ�����ݳ���
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @return ���ջ����еĿɶ�ȡ����������*/

    uint16              (*readData)(CeBlueHc* ceBlueHc, uint8* dataOutBuf, uint16 readCount);/*!<
                                                                             @brief �ӽ��ջ����ж�ȡ����
                                                                             @param ceBlueHc:CeBlueHc���Զ���ָ��
                                                                             @param dataOutBuf:��ȡ���ݴ�ŵĻ���
                                                                             @param readCount:��Ҫ��ȡ�����ݳ���
                                                                             @return ʵ�ʶ�ȡ�������ݳ���*/
}CeBlueHcOpBase;
/*
 *CeBlueHc��������ʵ��
 */
extern const CeBlueHcOpBase ceBlueHcOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BLUE_HC_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__CE_BLUE_HC_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function xxxxxzzzz
******************************************************************************

#include "Creelinks.h"
#include "CeBlueHc.h"
#include "CeTft320Nt.h"

CeTft320Nt ceTft320Nt;
CeBlueHc ceBlueHc;


int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                  //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceTft320NtOp.initial(&ceTft320Nt,R26L36);
    ceDebugOp.registerAppendString(ceTft320NtOp.appendString);
    ceBlueHcOp.initial(&ceBlueHc,R18Uart,R10TI2c);

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        while(CE_STATUS_SUCCESS != ceBlueHcOp.parmentConfig(&ceBlueHc,CE_BLUE_HC_WORK_MODE_MASTER,CE_BLUE_HC_DEV_TYPE_NO_STANDARD,"haha","1234"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("parmentConfig Faile \n");
        }

        while(CE_STATUS_SUCCESS != ceBlueHcOp.checkDevIsExist(&ceBlueHc,"CC Technology"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("checkDevIsExist Faile \n");
        }

        while(CE_STATUS_SUCCESS != ceBlueHcOp.connectDevByName(&ceBlueHc,"CC Technology"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("connectDevByName Faile \n");
        }

        ceBlueHcOp.outParmentConfig(&ceBlueHc);

        while(1)
        {
            ceSystemOp.delayMs(1000);
            ceDebugOp.printf("status %d\n",ceBlueHcOp.getConnectStatus(&ceBlueHc));
        }
    };
}

******************************************************************************
*/

