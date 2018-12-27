/**
  ******************************************************************************
  * @file   CeFlash.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinksƽ̨��Flash�����ʵ�ֺ���������STM32F103RET6ƽ̨
  ******************************************************************************
  * @attention
  *
  *1)����Flash���õ�ͬһ��Flashת��ģ�飬��ȡѭ���ɼ���ʽת��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeFlash.h"
#ifdef __cplusplus
 extern "C" {
#endif  //__cplusplus

#ifdef __CE_USE_FLASH__

#define CE_FLASH_BASE_ADDRESS   (0x08000000 + CE_MCU_ROM_SIZE_KB - CE_FLASH_SIZE + 1)/*!< ����MCU��ROM��������Flash����ַ�����뱣֤CE_MCU_ROM_SIZE_KB��ֵ��ȷ*/

#ifdef __CE_CHECK_PAR__
/**
  * @brief  ����ȡ�����Ƿ�Ϸ�
  * @param  address:��ȡ��ַ
  * @param  dataOutBuf:��ȡ�����ݻ���
  * @param  readCount����ȡ�����ݳ���
  * @return  �Ϸ�����CE_STATUS_SUCCESS�����Ϸ�����CE_STATUS_PAR_ERROR
  */
CE_STATUS ceFlash_checkReadPar(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount)
{
    if ((addressIndex + 1) % CE_FLASH_READ_SIZE != 0 || dataOutBuf == CE_NULL || readCount% CE_FLASH_READ_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}

/**
* @brief  ���д������Ƿ�Ϸ�
* @param  address:д���ַ
* @param  dataOutBuf:д������ݻ���
* @param  readCount��д������ݳ���
* @return  �Ϸ�����CE_STATUS_SUCCESS�����Ϸ�����CE_STATUS_PAR_ERROR
*/
CE_STATUS ceFlash_checkWritePar(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount)
{
    if ((addressIndex + 1) % CE_FLASH_WRITE_SIZE != 0 || dataInBuf == CE_NULL || writeCount% CE_FLASH_WRITE_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}

/**
* @brief  �����������Ƿ�Ϸ�
* @param  address:������ַ
* @param  readCount�����������ݳ���
* @return  �Ϸ�����CE_STATUS_SUCCESS�����Ϸ�����CE_STATUS_PAR_ERROR
*/
CE_STATUS ceFlash_checkErasePar(uint32 addressIndex, uint32 wipeCount)
{
    if ((addressIndex + 1) % CE_FLASH_ERASE_SIZE != 0 ||  wipeCount% CE_FLASH_WRITE_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif

/**
* @brief   ��ϵͳ���õĳ�ʼ��Flashת��
* @return  ϵͳ״̬�룬���ܵķ���ֵ:CE_STATUS_SUCCESS,CE_STATUS_INITIAL_FALSE
*/
CE_STATUS ceFlash_initialBySystem(void)
{
    return CE_STATUS_SUCCESS;//��STM32F103�У�Ϊ�ա���������������ʵ������ƶ�
}

/**
  * @brief ��Flash�ж�����
  * @param address:Ҫ��ȡ���׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_READ_SIZE��������-1
  * @param dataOutBuf:��ȡ�����ݴ�ŵĻ�����
  * @param readCount:Ҫ��ȡ�����ݳ��ȣ����� CE_FLASH_READ_SIZE�������������ز�������
  * @return ��ȡ���ݻ���
  */
uint8* ceFlash_read(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount)
{
    uint32 i = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkReadPar( addressIndex, dataOutBuf,  readCount));
#endif //__CE_CHECK_PAR__

    for(i = 0; i < readCount; i+= 2)//��֪STM32F103һ����С��ȡ����2���ֽ�
    {
        uint16 readVal = *(volatile uint16*)(addressIndex+ CE_FLASH_BASE_ADDRESS + i);
        dataOutBuf[i] = (uint8)((readVal >> 8) & 0xFF);
        dataOutBuf[i+1] = (uint8)((readVal>> 0) & 0xFF);
    }
    return dataOutBuf;
}

/**
  * @brief ��Flash��д���ݣ���Ҫ��֤�������ѱ�����(�������е�����λ(bit)������Ϊ1)�����򷵻ز���ʧ�ܣ�
  * @param address:Ҫ������׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_WRITE_SIZE ��������-1�����򷵻ز�������
  * @param dataInBuf:Ҫд������ݻ�����
  * @param writeCount:Ҫд������ݳ��ȣ���Χ0��(CE_FLASH_DATA_SIZE - 1)�������� CE_FLASH_WRITE_SIZE ��������-1�����ز�������
  * @return ����CE_SUCCESS���ʾд��ɹ������� CE_STATUS_PAR_ERROR ��ʾ�������󣬷��� CE_STATUS_FAILE ��ʾ����ʧ��
  */
CE_STATUS ceFlash_write(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount)
{
    uint32 i = 0;
    #ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkWritePar(addressIndex, dataInBuf, writeCount));
    #endif //__CE_CHECK_PAR__

    FLASH_Unlock();         //����д����
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for (i = 0; i < writeCount; i += 2)
    {
        FLASH_ProgramHalfWord((addressIndex + CE_FLASH_BASE_ADDRESS + i), ((uint16)(dataInBuf[i]) << 8 | (uint16)(dataInBuf[i + 1])));
    }
    FLASH_Lock();//����д����
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  ��Flashָ����ַ���в�������
  * @param  address:Ҫ�������׵�ַ����Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_ERASE_SIZE ��������-1�����򷵻ز�������
  * @param  wipeCount:Ҫ�����ĳ��ȣ���Χ0��(CE_FLASH_DATA_SIZE - 1)���ұ���Ϊ CE_FLASH_ERASE_SIZE ��������-1�����򷵻ز�������
  * @return  ����CE_SUCCESS���ʾ�����ɹ�������CE_STATUS_FAILE��ʾ����ʧ��
  */
CE_STATUS ceFlash_erase(uint32 addressIndex, uint32 eraseCount)
{
    #ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkErasePar(addressIndex, eraseCount));
    #endif //__CE_CHECK_PAR__
    FLASH_Unlock();         //����д����
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(CE_FLASH_BASE_ADDRESS + eraseCount);//�����������
    FLASH_Lock();//����д����
    return CE_STATUS_SUCCESS;
}

const CeFlashOp ceFlashOp = {ceFlash_write, ceFlash_read, ceFlash_erase };

#endif  //__CE_USE_FLASH__

#ifdef __cplusplus
 }
#endif  //__cplusplus
