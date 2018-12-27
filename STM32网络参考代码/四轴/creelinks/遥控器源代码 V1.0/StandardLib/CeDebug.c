/**
  ******************************************************************************
  * @file    CeDeBug.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeDeBugģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeDebug.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

CeDebug ceDebug;

const char pHexIndex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };  /*!< Hex�ļ���*/
/**
  * @brief   ��ʹ�õ�UART���д����ʽ������initial��ִ�д˺�����ceUart:ʹ���ĸ�UART��Դ��Ϊ��ʽ��
  * @param   ceUart:��ӡ������Ϣʹ�õ�Uart��Դ��
  * @return  None
  */
CE_STATUS ceDebug_initial(CE_RESOURCE ceUart)
{
    ceDebug.ceUart.ceResource = CE_NULL_RESOURCE;
    ceDebug.isPrintfFinish = 0x01;
    ceDebug.appendStringBufIndex = 0x00;    
    
    if(ceUart == CE_NULL_RESOURCE)
    {
        return CE_STATUS_SUCCESS;
    }else if ((ceUart & CE_RES_MARK_UART) != CE_RES_MARK_UART)
    {

        return CE_STATUS_RESOURCE_ERROR;
    }

        
        
#ifdef __CE_USE_DEBUG__
    ceDebug.ceUart.ceResource = ceUart;
    ceDebug.ceUart.uartBaudRate = CE_UART_BAUD_RATE_115200;
    ceDebug.ceUart.uartParity = CE_UART_PARITY_NO;
    ceDebug.ceUart.uartStopBits = CE_UART_STOP_BITS_1;
    ceDebug.ceUart.uartWordLength = CE_UART_WORD_LENGTH_8B;
    ceDebug.ceUart.recvBuf = ceDebug.recvBuf;
    ceDebug.ceUart.recvBufSize = CE_DEBUG_RECV_BUF_SIZE;
    ceUartOp.initial(&(ceDebug.ceUart));
    ceUartOp.start(&(ceDebug.ceUart));
#endif 
    return CE_STATUS_SUCCESS;
}


/**
  * @brief  ���û�ʹ�ö������ʾ�豸ʱ�����Խ���ʾ�豸��appendStringʹ�ô˺���ע�ᣬע����ɺ�ɴﵽ��ʾ�豸��ʾ������Ϣ������
  * @param  appendString:��Ҫע���appendString����
  * @return None
  */
void ceDebug_registerAppendString(void (appendString)(const char* msg))
{
    #ifdef __CE_USE_DEBUG__
    if(ceDebug.ceUart.ceResource != CE_NULL_RESOURCE)
    {
        ceDebug.appendString = appendString;

        ceDebugOp.printf("\\**********************************\\\n\n");
        ceDebugOp.printf("\\*WELCOME TO USE CREELINKS PLATFORM*\\\n");
        ceDebugOp.printf("\\*Hardware platform:ELinkSTM/ELinkSTM_Pro*\\\n");
        ceDebugOp.printf("\\*Standard library version:V17*\\\n");
        ceDebugOp.printf("\\*Core library version:V17*\\\n");
        ceDebugOp.printf("\\*If the software used for commercial purposes, please contact the software development team.*\\\n");
        ceDebugOp.printf("\\*Beijing Darcern technology co., LTD*\\\n");
        ceDebugOp.printf("\\*CREELINKS DEVELOPMENT TEAM*\\\n\n");
        ceDebugOp.printf("\\***********************************\\\n\n");
    }
    #endif
}

/**
  * @brief  ȡ���ڶ�����ʾ�豸����ʾ������Ϣ����
  * @return  None
  */
void ceDebug_unRegisterAppendString(void)
{
    ceDebug.appendString = CE_NULL;
}

/**
  * @brief   ͨ��UART�ڴ�ӡchar�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfChar(char val, char* buf, uint16* index)
{
    if (buf == CE_NULL)
    {
        #ifdef __CE_USE_DEBUG__
        if(ceDebug.ceUart.ceResource != CE_NULL_RESOURCE)
        {
            ceUartOp.sendData(&ceDebug.ceUart, (uint8*)(&val), 1);
            if (ceDebug.appendString != CE_NULL)
            {
                ceDebug.appendStringBuf[ceDebug.appendStringBufIndex] = val;
                ceDebug.appendStringBufIndex++;
            }
        }
        #endif 
    }
    else
    {
        buf[*index] = val;
        (*index)++;
        buf[*index] = '\0';
    }
}

/**
  * @brief   ͨ��UART�ڴ�ӡInt32�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfInt32(int32 val, char* buf, uint16* index)
{
    char rList[10];
    int32 i;
    uint8 isP = 0;
    if (val == 0)
    {
        ceDebug_printfChar('0',buf,index);
        return;
    }
    if (val < 0)
    {
        ceDebug_printfChar('-', buf, index);
        val = -val;
    }

    for (i = 0; i < 10; i++)
    {
        rList[i] = val % 10;
        val /= 10;
    }

    for (i = 9; i >= 0; i--)
    {
        if (rList[i] != 0x00)
        {
            isP = 0x01;
        }
        if (isP == 0x01)
        {
            rList[i] += 0x30;
            ceDebug_printfChar(rList[i], buf, index);
        }
    }
}

/**
  * @brief   ͨ��UART�ڴ�ӡUint32�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfUint32(uint32 val, char* buf, uint16* index)
{
    char rList[10];
    int i;
    uint8 isP = 0;
    if (val == 0)
    {
        ceDebug_printfChar('0', buf, index);
        return;
    }
    for (i = 0; i < 10; i++)
    {
        rList[i] = val % 10;
        val /= 10;
    }

    for (i = 9; i >= 0; i--)
    {
        if (rList[i] != 0x00)
        {
            isP = 0x01;
        }
        if (isP == 0x01)
        {
            rList[i] += 0x30;
            ceDebug_printfChar(rList[i], buf, index);
        }
    }
}

/**
  * @brief   ͨ��UART�ڴ�ӡFp64�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfFp64(fp64 val, char* buf, uint16* index)
{
    int tmpint = 0;
    if (val < 0)
    {
        ceDebug_printfChar('-', buf, index);
        val = -val;
    }
    tmpint = (int) val;
    ceDebug_printfUint32(tmpint, buf, index);
    ceDebug_printfChar('.', buf, index);
    val = val - tmpint;
    tmpint = (int) (val * 10000000);//������Bug�����valΪ0.004567��ʱ�򣬳˷�֮��ὫС������������0��ȥ�ˣ��Ժ�������־��
    ceDebug_printfUint32(tmpint, buf, index);
}


/**
  * @brief   ͨ��UART�ڴ�ӡStr�����裬�ڲ�����
  * @param   str:����ӡ�����ݵ�ָ��
  * @return  None
  */
void ceDebug_printfStr(char* str, char* buf, uint16* index)
{
    while (*str)
    {
        ceDebug_printfChar(*str++, buf, index);
    }
}

/**
  * @brief   ͨ��UART�ڴ�ӡBin�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfBin(uint32 val, char* buf, uint16* index)
{
    int i;
    for (i = 31; i >= 0; i--)
    {
        ceDebug_printfChar((char) ((val >> i) & 0x01) + 0x30, buf, index);
    }
}


/**
  * @brief   ͨ��UART�ڴ�ӡHex�����裬�ڲ�����
  * @param   val:����ӡ������
  * @return  None
  */
void ceDebug_printfHex(uint32 val, char* buf, uint16* index)
{
    ceDebug_printfChar(pHexIndex[(val >> 28) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 24) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 20) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 16) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 12) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 8) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 4) & 0x0F], buf, index);
    ceDebug_printfChar(pHexIndex[(val >> 0) & 0x0F], buf, index);
}

/**
  * @brief   ͨ��UART�ڴ�ӡ��ʽ���ĵ�����Ϣ�����裬���ڵ���initialDebug�󣬲ſ�������ʹ��
  * @param   fmt:����ӡ�ĸ�ʽ������
  * @return  None
  */
void ceDebug_printf(const char* fmt, ...)
{
    if (ceDebug.ceUart.ceResource != CE_NULL_RESOURCE)//ִ��initialDebug����
    {
        #ifdef __CE_USE_DEBUG__
        int32 vargint = 0;
        uint32 varguint = 0;
        char* vargpch = CE_NULL;
        char vargch = 0;
        fp64 vargflt = 0;
        const char* pfmt = CE_NULL;
        char* vp = CE_NULL;
        uint16 tickTemp = 0;
        ceDebug.appendStringBufIndex = 0;
        
        while(ceDebug.isPrintfFinish == 0x00)
        {
            ceSystemOp.delayMs(5);
            tickTemp++;
            if(tickTemp >= 65535)
            {
                ceDebug.isPrintfFinish = 0x01;
            }
        }
        ceDebug.isPrintfFinish = 0x00;
        vp = (char*) (&(fmt)) + ((sizeof(fmt) + sizeof(int) - 1) & ~(sizeof(int) - 1));//va_start(vp, fmt);
        pfmt = fmt;
        while (*pfmt)
        {
            if (*pfmt == '%')
            {
                switch (*(++pfmt))
                {
                case 'c':
                    vargch = (*(int *) ((vp += ((sizeof(int) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(int) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfChar(vargch,CE_NULL, CE_NULL);
                    break;
                case 'd':
                case 'i':
                    vargint = (*(int32 *) ((vp += ((sizeof(int32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(int32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfInt32(vargint, CE_NULL, CE_NULL);
                    break;
                case 'u':
                case 'U':
                    varguint = (*(uint32 *) ((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfUint32((uint32) varguint, CE_NULL, CE_NULL);
                    break;
                case 'f':
                    vargflt = (*(fp64 *) ((vp += ((sizeof(fp64) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(fp64) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfFp64(vargflt, CE_NULL, CE_NULL);
                    break;
                case 's':
                    vargpch = (*(char* *) ((vp += ((sizeof(char*) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(char*) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfStr(vargpch, CE_NULL, CE_NULL);
                    break;
                case 'b':
                case 'B':
                    vargint = (*(uint32 *) ((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfBin(vargint, CE_NULL, CE_NULL);
                    break;
                case 'x':
                case 'X':
                    vargint = (*(uint32 *) ((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                    ceDebug_printfHex(vargint, CE_NULL, CE_NULL);
                    break;
                case '%':
                    ceDebug_printfChar('%', CE_NULL, CE_NULL);
                    ceDebug_printfChar('%', CE_NULL, CE_NULL);//��Ϊǰһ����%�����ܵ������������Ըô�ӡ����%��
                    break;
                default:
                    ceDebug_printfStr("!Unknown format!", CE_NULL, CE_NULL);
                    break;
                }
                pfmt++;
            }
            else
            {
                ceDebug_printfChar(*pfmt++, CE_NULL, CE_NULL);
            }
        }
        vp = (char*) 0;//va_end(vp);
        ceDebug.isPrintfFinish = 0x01;
        
        if (ceDebug.appendString != CE_NULL)
        {
            ceDebug.appendStringBuf[ceDebug.appendStringBufIndex] = '\0';
            ceDebug.appendString(ceDebug.appendStringBuf);
        }
        #endif //defined(__CE_USE_DEBUG__)
    }
}


/**
  * @brief   �Ѹ�ʽ��������ƴ�ӵ��ַ�����������
  * @param   buffer:��ƴ�ӵ��ַ���������
  * @param   format:��ƴ�ӵĸ�ʽ������
  * @return  None
  */
int ceDebug_sprintf(char *buffer, const char *format, ...)
{
    int32 vargint = 0;
    uint32 varguint = 0;
    char* vargpch = CE_NULL;
    char vargch = 0;
    fp64 vargflt = 0;
    const char* pfmt = CE_NULL;
    char* vp = CE_NULL;
    uint16 tickTemp = 0;
    uint16 index = 0;
    while (ceDebug.isPrintfFinish == 0x00)
    {
        ceSystemOp.delayMs(5);
        tickTemp++;
        if (tickTemp >= 65535)
        {
            ceDebug.isPrintfFinish = 0x01;
        }
    }
    ceDebug.isPrintfFinish = 0x00;
    vp = (char*)(&(format)) + ((sizeof(format) + sizeof(int) - 1) & ~(sizeof(int) - 1));//va_start(vp, fmt);
    pfmt = format;
    while (*pfmt)
    {
        if (*pfmt == '%')
        {
            switch (*(++pfmt))
            {
            case 'c':
                vargch = (*(int *)((vp += ((sizeof(int) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(int) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfChar(vargch,buffer, &index);
                break;
            case 'd':
            case 'i':
                vargint = (*(int32 *)((vp += ((sizeof(int32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(int32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfInt32(vargint, buffer, &index);
                break;
            case 'u':
            case 'U':
                varguint = (*(uint32 *)((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfUint32((uint32)varguint, buffer, &index);
                break;
            case 'f':
                vargflt = (*(fp64 *)((vp += ((sizeof(fp64) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(fp64) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfFp64(vargflt, buffer, &index);
                break;
            case 's':
                vargpch = (*(char* *)((vp += ((sizeof(char*) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(char*) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfStr(vargpch, buffer, &index);
                break;
            case 'b':
            case 'B':
                vargint = (*(uint32 *)((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfBin(vargint, buffer, &index);
                break;
            case 'x':
            case 'X':
                vargint = (*(uint32 *)((vp += ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))) - ((sizeof(uint32) + sizeof(int) - 1) & ~(sizeof(int) - 1))));
                ceDebug_printfHex(vargint, buffer, &index);
                break;
            case '%':
                ceDebug_printfChar('%', buffer, &index);
                ceDebug_printfChar('%', buffer, &index);//��Ϊǰһ����%�����ܵ������������Ըô�ӡ����%
                break;
            default:
                ceDebug_printfStr("!Unknown format!", buffer, &index);
                break;
            }
            pfmt++;
        }
        else
        {
            ceDebug_printfChar(*pfmt++, buffer, &index);
        }
    }
    vp = (char*)0;//va_end(vp);
    ceDebug.isPrintfFinish = 0x01;

    return (index);
}


/**
  * @brief   ʹ��Uart���е���ʱ�������λ���������Ŀ��õ�����������
  * @return  ���õ���������
  */
uint8 ceDebug_getRecvDataCount(void)
{
    #ifdef __CE_USE_DEBUG__
    if(    ceDebug.ceUart.ceResource != CE_NULL_RESOURCE)
        return ceFifoOp.getCanReadSize(&(ceDebug.ceUart.ceExFifo));
    else 
        return 0;
    #else
    return 0;
#endif 
}

/**
  * @brief   ʹ��Uart���е���ʱ�������λ���������ĵ�������
  * @param   dataOutBuf:�����ȡ���ݵĻ���
  * @param   readCount:��Ҫ��ȡ�����ݳ���
  * @return  ʵ�ʶ�ȡ�������ݳ���
  */
uint8 ceDebug_getRecvData(uint8* dataOutBuf, uint8 readCount)
{
    #ifdef __CE_USE_DEBUG__
    if(ceDebug.ceUart.ceResource != CE_NULL_RESOURCE)
        return ceFifoOp.read(&(ceDebug.ceUart.ceExFifo), dataOutBuf, readCount);
    else 
        return 0;
    #else
    return 0;
    #endif
}
/**
  * @brief  CeDeBugģ�����������
  */
const CeDebugOp ceDebugOp = {ceDebug_initial,ceDebug_registerAppendString,ceDebug_unRegisterAppendString,ceDebug_printf,ceDebug_sprintf,ceDebug_getRecvDataCount,ceDebug_getRecvData};

#ifdef __cplusplus
 }
#endif //__cplusplus
