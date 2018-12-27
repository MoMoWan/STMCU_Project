/**
  ******************************************************************************
  * @file    CeI2c.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeI2c��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)��ƽ̨��ͬ������I2c���߲������ģ��ķ�ʽʵ�֣���ο�CeI2c.c�е�����
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_I2C_H__
#define __CE_I2C_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  ö�٣�I2cMaster��������
  */
typedef enum
{
    CE_I2C_SPEED_100KBPS,                                                   /*!< I2C�������ʣ�100Kbps*/
    CE_I2C_SPEED_400KBPS,                                                   /*!< I2C�������ʣ�400Kbps*/
    CE_I2C_SPEED_3_4MBPS,                                                   /*!< I2C�������ʣ�3.4Mbps*/
}CE_I2C_MASTER_SPEED;

/**
  * @brief  �ṹ�壬I2cMaster����������Լ���
  */
typedef struct
{
    CE_RESOURCE         ceResource;                                         /*!< I2C��Ӧ����Դ��*/
    CE_I2C_MASTER_SPEED ceI2cMasterSpeed;                                   /*!< I2C���õ���������*/

    CeExI2cMasterPar    ceExI2cMasterPar;                                   /*!< �봦����ƽ̨��صĶ�������ṹ�壬������ߴ���Ч�ʣ��û������ע*/
}CeI2cMaster;

/**
  * @brief  �ṹ�壬I2cMaster������ò�������
  */
typedef struct
{
    CE_STATUS   (*initial)(CeI2cMaster* ceI2cMaster);                       /*!< @brief ��ʼ��I2C����
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

    void        (*start)(CeI2cMaster* ceI2cMaster);                         /*!< @brief ��ʼI2C����
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

    void        (*stop)(CeI2cMaster* ceI2cMaster);                          /*!< @brief ֹͣI2C����
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

    void        (*writeByte)(CeI2cMaster* ceI2cMaster,uint8 val);   /*!< @brief ����һ���ֽڵ�I2C����
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��
                                                                                 @param val:Ҫ���͵����ݣ���λ�ֽ�*/

    uint8       (*readByte)(CeI2cMaster* ceI2cMaster,uint8 isAck); /*!< @brief ����һ���ֽڵ�I2C����
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��
                                                                                 @param isAck:�Ƿ��ڽ�����ɺ󣬷���Ӧ���ź�
                                                                                 @return �����յ�������*/

    CE_STATUS   (*waitAck)(CeI2cMaster* ceI2cMaster);              /*!< @brief �ȴ����豸��Ӧ���ź�
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

    CE_STATUS   (*lockBus)(CeI2cMaster* ceI2cMaster);                       /*!< @brief ��ȡI2c���߿���Ȩ
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

    void        (*unlockBus)(CeI2cMaster* ceI2cMaster);                     /*!< @brief �ͷ�I2c���߿���Ȩ
                                                                                 @param ceI2cMaster:I2cMaster���Զ���ָ��*/

}CeI2cMasterOp;
extern const CeI2cMasterOp ceI2cMasterOp;                               /*!< ������I2C��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_I2C_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ����I2c����д���ݣ���ϸʾ���ɲο�ʹ�õ�I2c�ӿڵ�ģ�飩
******************************************************************************
#include "Creelinks.h"
CeI2cMaster myI2c;                                  //����I2c���Զ���
uint8 recvData;                                         //�������ݻ���
int main(void)
{
    ceSystemOp.initial();                           //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                            //ͨ��Uart�������Debug��Ϣ����λ��
                                                    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    myI2c.ceResource = RxI2c;                       //ָ��I2cʹ�õ���Դ��
    myI2c.ceI2cMasterSpeed = CE_I2C_SPEED_3_4MBPS;  //�趨I2c�Ĺ�������
    if (ceI2cMasterOp.initia(&myI2c) != CE_STATUS_SUCCESS)  //��ʼ��I2c
    {
        ceDebugOp.printf("I2c initial fail!\n");
    }
    ceDebugOp.printf("I2c initial success!\n");
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
                                                    //TODO:���ڴ˴������û�����
        ceI2cMasterOp.lockBus(&myI2c);              //���I2c���߿���Ȩ����ΪI2c�����Ͽ��ܹ��ж�����豸������������ͬʱʹ��I2c���ߣ�
        ceI2cMasterOp.start(&myI2c);                //��ʼ����
        ceI2cMasterOp.writeByte(&myI2c, 0x21);//��������
        recvData = ceI2cMasterOp.readByte(&myI2c, 0x00);//��ȡ���ݣ�0x00��ʾ����Ҫ���豸Ӧ���ź�
        ceDebugOp.printf("I2c recv data:%d\n", recvData);//����λ�����͵�����Ϣ
        ceI2cMasterOp.stop(&myI2c);                 //ֹͣI2c����
        ceI2cMasterOp.unlockBus(&myI2c);            //�ͷ����߿���Ȩ
    };
}
******************************************************************************
*/
