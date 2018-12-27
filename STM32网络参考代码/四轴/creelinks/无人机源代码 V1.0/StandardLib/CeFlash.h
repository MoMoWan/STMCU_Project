/**
  ******************************************************************************
  * @file   CeFlash.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinksƽ̨Flash����Ĳ���ͷ�ļ�
  ****************************************************************************** 
  * @attention
  *
  *1)��ͬFlash�Ķ�ȡ��д�롢�����Ȳ���Ҫ��Ŀ�/����/ҳ��С��һ�£��û�����ϸ�鿴
  *  CeMcu.h��CE_FLASH_SIZE��CE_FLASH_READ_SIZE��CE_FLASH_WRITE_SIZE��
  *  CE_FLASH_ERASE_SIZE ��ֵ��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_FLASH_H__
#define __CE_FLASH_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  �ṹ�壬FLASH������ò�������
  */
typedef struct
{
    CE_STATUS   (*write)(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount);/*!<
                                                     @brief ��Flash��д���ݣ���Ҫ��֤�������ѱ�����������д�����ʧ�ܣ�
                                                     @param address:Ҫ������׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_WRITE_SIZE��������-1�����򷵻ز�������
                                                     @param dataInBuf:Ҫд������ݻ�����
                                                     @param writeCount:Ҫд������ݳ��ȣ������� CE_FLASH_WRITE_SIZE��������-1�����򷵻ز�������
                                                     @return ����CE_SUCCESS���ʾд��ɹ������� CE_STATUS_PAR_ERROR ��ʾ�������󣬷��� CE_STATUS_FAILE ��ʾ����ʧ��*/

    uint8*      (*read)(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount);/*!<
                                                     @brief ��Flash�ж�����
                                                     @param address:Ҫ��ȡ���׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_READ_SIZE��������-1
                                                     @param dataOutBuf:��ȡ�����ݴ�ŵĻ�����
                                                     @param readCount:Ҫ��ȡ�����ݳ��ȣ����� CE_FLASH_READ_SIZE��������-1�����ز�������
                                                     @return ��ȡ���ݻ���*/

    CE_STATUS   (*erase)(uint32 addressIndex, uint32 eraseCount);/*!<
                                                     @brief ��Flashָ����ַ���в�������
                                                     @param address:Ҫ�������׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_ERASE_SIZE��������-1�����򷵻ز�������
                                                     @param eraseCount:Ҫ�����ĳ��ȣ������� CE_FLASH_ERASE_SIZE��������-1�����򷵻ز�������
                                                     @return ����CE_SUCCESS���ʾ�����ɹ�������CE_STATUS_FAILE��ʾ����ʧ��*/
}CeFlashOp;
extern const CeFlashOp ceFlashOp;          /*!< ������Flash��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_FLASH_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����)
* @function ��ȡFlash��ת��ֵ����ͨ��Uart�ڴ�������ڵ�������
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
