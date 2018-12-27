/**
  ******************************************************************************
  * @file    CeExtra.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   �봦����ƽ̨�޹ص�CeExtra��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeExtra.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief   �Բ���ȡ����ֵ
  * @param   val:��Ҫ�����ֵ
  * @return  ������
  */
fp32 ceAbs(fp32 val)
{
    if(val < 0)
    {
        return -val;
    }else
    {
        return val;
    }
}

const CeMathOp ceMathOp = {ceAbs};

/**
  * @brief   �����ַ������ȣ�������'\0'
  * @param   str:��Ҫ�����ֵ
  * @return  �ַ�������
  */
uint32 ceStrlen(const char* str)
{
    uint32 i=0;
    while(1)
    {
        if(str[i] == '\0')
        {
            return i;
        }
        i++;
    }
}

/**
  * @brief   �ַ�������
  * @param   dest:������Ŀ���ַ���
  * @param   src:������Դ�ַ���
  * @return  ����Ŀ���ַ����ĵ�ַ
  */
char*  ceStrcpy(char *dest, char *src)
{
    uint16 i;
    for (i = 0; i < 65535; i++)
    {
        dest[i] = src[i];
        if (src[i] == '\0')
        {
            break;
        }
    }
    return dest;
}

/**
  * @brief   �ַ����Ƚ�
  * @param   str1:���Ƚϵ��ַ���1
  * @param   str2:���Ƚϵ��ַ���2
  * @return  �������ַ�����ͬ����0����ͬ����-1
  */
int8 ceStrcmp(const char* str1, const char* str2)
{
    uint16 i;
    for (i = 0; i < 65535; i++)
    {
        if (str1[i] == str2[i])
        {
            if (str1[i] == '\0')
            {
                return 0;//is same
            }
        }
        else
        {
            if (str1[i] == '\0' || str2[i] == '\0')
            {
                return -1;
            }
        }
    }
    return 0;
}

/**
  * @brief   ������ת��Ϊ�ַ���
  * @param   val:����
  * @param   str:�ַ�������
  * @param   radix:��Ҫת���Ľ���
  * @return  ת����ɺ���ַ���
  */
char* ceItoa(int32 val, char* charBuff, uint8 radix)
{
    char index[] = "0123456789ABCDEF";
    unsigned unum;/*�м����*/
    char temp;
    int i = 0, j, k;
    /*ȷ��val��ֵ*/
    if (radix == 10 && val < 0)/*ʮ���Ƹ���*/
    {
        unum = (unsigned) -val;
        charBuff[i++] = '-';
    }
    else
        unum = (unsigned) val;/*�������*/
    /*ת��*/
    do
    {
        charBuff[i++] = index[unum % (unsigned) radix];
        unum /= radix;
    } while (unum);
    charBuff[i] = '\0';
    /*����*/
    if (charBuff[0] == '-')
        k = 1;/*ʮ���Ƹ���*/
    else
        k = 0;

    for (j = k; j <= (i - 1) / 2; j++)
    {
        temp = charBuff[j];
        charBuff[j] = charBuff[i - 1 + k - j];
        charBuff[i - 1 + k - j] = temp;
    }
    return charBuff;

}

/**
  * @brief   ���ַ���ת��Ϊ����
  * @param   str:Ҫת�����ַ���
  * @return  ת���������
  */
int32 ceAtoi(const char* str)
{
    uint8  bMinus = 0x00;
    int result = 0;
    if (('0' > *str || *str > '9') && (*str == '+' || *str == '-'))
    {
        if (*str == '-')
            bMinus = 0x01;
        str++;
    }
    while (*str != '\0')
    {
        if ('0' > *str || *str > '9')
            break;
        else
            result = result * 10 + (*str++ - '0');
    }

    if (*str != '\0')//no-normal end
        return -2;

    return (bMinus == 0x01) ? -result : result;
}

const CeStringOp ceStringOp = {ceStrlen, ceStrcmp,ceStrcpy, ceItoa, ceAtoi};


/**
  * @brief   Fifo��ʼ��
  * @param   ceFifo:Cefifo��������ָ��
  */
void ceFifo_initial(CeFifo* ceFifo)
{
    ceFifo->readIndex = 0;
    ceFifo->writeIndex = 1;
    ceFifo->isReadLock = 0x00;
    ceFifo->isWriteLock = 0x00;
}

/**
  * @brief   ���Fifo�Ƿ�Ϊ�գ����Ƿ������ݿɶ�
  * @param   ceFifo:Cefifo��������ָ��
  * @return  Fifo�Ƿ�Ϊ�գ�����0��Fifo��Ϊ�գ�����1��FifoΪ��
  */
uint8 ceFifo_isEmpty(CeFifo* ceFifo)
{
    if (ceFifo->writeIndex  - ceFifo->readIndex== 1 || (ceFifo->readIndex == ceFifo->buffSize-1 && ceFifo->writeIndex == 0))
    {
        return 0x01;
    } else
    {
        return 0x00;
    }
}

/**
  * @brief   ��ÿɶ������ݳ���
  * @param   ceFifo:Cefifo��������ָ��
  * @return  �ɶ������ݳ���
  */
uint16 ceFifo_getCanReadSize(CeFifo* ceFifo)
{
    uint16 temp = 0;
    if (ceFifo->readIndex < ceFifo->writeIndex)
    {
        temp = ceFifo->writeIndex - ceFifo->readIndex - 1;
    } else
    {
        temp = (ceFifo->writeIndex - 0) + (ceFifo->buffSize - ceFifo->readIndex - 1);
    }
    return temp;
}

/**
  * @brief   ��ÿ�д������ݳ���
  * @param   ceFifo:Cefifo��������ָ��
  * @return  ��д������ݳ���
  */
uint16 ceFifo_getCanWriteSize(CeFifo* ceFifo)
{
    uint16 temp = 0;
    if(ceFifo->readIndex > ceFifo->writeIndex)
    {
        temp = ceFifo->readIndex - ceFifo->writeIndex -1;
    }else
    {
        if(ceFifo->readIndex == 0)
        {
            temp = ceFifo->buffSize- ceFifo->writeIndex-1;
        }else
        {
            temp = (ceFifo->buffSize- ceFifo->writeIndex-1) + (ceFifo->readIndex-0);
        }
    }
    return temp;
}

/**
  * @brief   ��Fifo��д������
  * @param   ceFifo:Cefifo��������ָ��
  * @param   dataInBuf:��д������ݻ���
  * @param   dataInCount:��д������ݳ���
  * @return  ʵ��д������ݳ��ȣ�������Fifo����
  */
uint16 ceFifo_write(CeFifo* ceFifo, uint8* dataInBuf, uint16 dataInCount)
{
    uint16 temp = 0;
    while (ceFifo->isWriteLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isWriteLock = 0x01;
    while(1)
    {
        if(ceFifo->writeIndex < ceFifo->readIndex)
        {
            if(ceFifo->writeIndex == ceFifo->readIndex-1)
            {
                break;
            }
        }else
        {
            if((ceFifo->writeIndex == ceFifo->buffSize-1) && (ceFifo->readIndex == 0))
            {
                break;
            }
        }
        ceFifo->buff[ceFifo->writeIndex] = dataInBuf[temp];
        temp++;
        ceFifo->writeIndex++;
        if(ceFifo->writeIndex == ceFifo->buffSize)
        {
            ceFifo->writeIndex = 0;
        }
        if(temp >= dataInCount)
        {
            break;
        }
    }
    ceFifo->isWriteLock = 0x00;
    return temp;

}

/**
  * @brief   ��Fifo�ж�ȡ����
  * @param   ceFifo:Cefifo��������ָ��
  * @param   dataOutBuf:��ȡ��������ŵĻ���
  * @param   dataOutCount:��Ҫ��ȡ�����ݳ���
  * @return  ʵ�ʶ�ȡ�����ݳ���
  */
uint16 ceFifo_read(CeFifo* ceFifo, uint8* dataOutBuf, uint16 dataOutCount)
{
    uint16 temp = 0;
    while (ceFifo->isReadLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isReadLock = 0x01;

    while(ceFifo->readIndex != ceFifo->writeIndex-1 && !(ceFifo->readIndex== ceFifo->buffSize-1 && ceFifo->writeIndex == 0))
    {
        if(ceFifo->readIndex < ceFifo->writeIndex)
        {
            if(ceFifo->readIndex == ceFifo->writeIndex-1)
            {
                break;
            }
        }else
        {
            if((ceFifo->readIndex == ceFifo->buffSize-1) && (ceFifo->writeIndex == 0))
            {
                break;
            }
        }
        ceFifo->readIndex++;
        if(ceFifo->readIndex == ceFifo->buffSize)
        {
            ceFifo->readIndex = 0;
        }
        dataOutBuf[temp] = ceFifo->buff[ceFifo->readIndex];
        temp++;
        if(temp >= dataOutCount)
        {
            break;
        }
    }
    ceFifo->isReadLock = 0x00;
    return temp;
}

/**
  * @brief   ���Fifo�е�����
  * @param   ceFifo:Cefifo��������ָ��
  */
void ceFifo_clear(CeFifo* ceFifo)
{
    while (ceFifo->isReadLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isReadLock = 0x01;

    while (ceFifo->isWriteLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isWriteLock = 0x01;

    ceFifo->readIndex = 0;
    ceFifo->writeIndex = 1;

    ceFifo->isWriteLock = 0x00;
    ceFifo->isReadLock = 0x00;
}

/**
  * @brief   ���Fifo�Ƿ�����ʹ�õ�״̬
  * @param   ceFifo:Cefifo��������ָ��
  * @return  ����0��Fifoδ��ʹ�ã�1��Fifo���ڽ��ж���д����
  */
uint8   ceFifo_getReadLockStatus(CeFifo* ceFifo)
{
    return ceFifo->isWriteLock;
}

/**
  * @brief   ���Fifo�Ƿ���д����״̬
  * @param   ceFifo:Cefifo��������ָ��
  * @return  ����0��Fifoδ��ʹ�ã�1��Fifo���ڽ��ж���д����
  */
uint8   ceFifo_getWriteLockStatus(CeFifo* ceFifo)
{
    return ceFifo->isWriteLock;
}

const CeFifoOp ceFifoOp = {ceFifo_initial, ceFifo_isEmpty, ceFifo_getCanReadSize, ceFifo_getCanWriteSize, ceFifo_write, ceFifo_read, ceFifo_clear, ceFifo_getWriteLockStatus};


/**
  * @brief   Fifo��ʼ��
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  */
void ceDoubleFifo_initial(CeDoubleFifo* ceDoubleFifo)
{
    ceFifoOp.initial(&(ceDoubleFifo->ceFifoOne));
    ceFifoOp.initial(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   ���»��棬����һ��������Ϊ���ݸ��Ƶ��������浱��
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  */
void ceDoubleFifo_updata(CeDoubleFifo* ceDoubleFifo)//���һ��fifo���Ƿ���������Ҫ���Ƶ������С�
{
    uint16 count = ceFifoOp.getCanReadSize(&(ceDoubleFifo->ceFifoOne));
    if (count > 0)
    {
        uint16 i;
        uint8 temp;
        for (i = 0; i < count; i++)
        {
            if (ceFifoOp.getCanWriteSize(&ceDoubleFifo->ceFifoTwo) >= 1 && ceFifoOp.getWriteLockStatus(&(ceDoubleFifo->ceFifoTwo)) == 0x00)
            {
                ceFifoOp.read(&(ceDoubleFifo->ceFifoOne), &temp, 1);
                ceFifoOp.write(&(ceDoubleFifo->ceFifoTwo), &temp, 1);
            } else
            {
                break;
            }
        }
    }
}

/**
  * @brief   ���Fifo�Ƿ�Ϊ�գ����Ƿ������ݿɶ�
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  * @return  Fifo�Ƿ�Ϊ�գ�����0��Fifo��Ϊ�գ�����1��FifoΪ��
  */
uint8 ceDoubleFifo_isEmpty(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.isEmpty(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   ��ÿɶ������ݳ���
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  * @return  �ɶ������ݳ���
  */
uint16 ceDoubleFifo_getCanReadSize(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.getCanReadSize(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   ��ÿ�д������ݳ���
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  * @return  ��д������ݳ���
  */
uint16 ceDoubleFifo_getCanWriteSize(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.getCanWriteSize(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   ��Fifo��д������
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  * @param   dataInBuf:��д������ݻ���
  * @param   dataInCount:��д������ݳ���
  * @return  ʵ��д������ݳ��ȣ�������Fifo����
  */
uint16 ceDoubleFifo_write(CeDoubleFifo* ceDoubleFifo, uint8* dataInBuf, uint16 dataInCount)//�жϵ��ã��ܹ���֤״̬�����任
{
    if(ceDoubleFifo->ceFifoTwo.isWriteLock == 0x01)
    {
        return ceFifoOp.write(&(ceDoubleFifo->ceFifoOne),dataInBuf, dataInCount);
    }else
    {
        ceDoubleFifo_updata(ceDoubleFifo);
        return ceFifoOp.write(&(ceDoubleFifo->ceFifoTwo),dataInBuf, dataInCount);
    }
}

/**
  * @brief   ��Fifo�ж�ȡ����
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  * @param   dataOutBuf:��ȡ��������ŵĻ���
  *  @param  dataOutCount:��Ҫ��ȡ�����ݳ���
  * @return  ʵ�ʶ�ȡ�����ݳ���
  */
uint16 ceDoubleFifo_read(CeDoubleFifo* ceDoubleFifo, uint8* dataOutBuf, uint16 dataOutCount)//�̶ȵ��ã��������̿��ܻᱻ�жϴ��
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.read(&(ceDoubleFifo->ceFifoTwo),dataOutBuf, dataOutCount);
}

/**
  * @brief   ���Fifo�е�����
  * @param   ceDoubleFifo:CeDoubleFifo��������ָ��
  */
void ceDoubleFifo_clear(CeDoubleFifo* ceDoubleFifo)
{
    ceFifoOp.clear(&(ceDoubleFifo->ceFifoOne));
    ceFifoOp.clear(&(ceDoubleFifo->ceFifoOne));
}

const CeDoubleFifoOp ceDoubleFifoOp = {ceDoubleFifo_initial, ceDoubleFifo_isEmpty, ceDoubleFifo_getCanReadSize, ceDoubleFifo_getCanWriteSize,
                                            ceDoubleFifo_write, ceDoubleFifo_read, ceDoubleFifo_clear};

#ifdef __cplusplus
 }
#endif //__cplusplus
