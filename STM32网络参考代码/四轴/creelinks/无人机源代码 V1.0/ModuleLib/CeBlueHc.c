/**
  ******************************************************************************
  * @file    CeBlueHc.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeBlueHcģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBlueHc.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define CE_BLUE_HC_BUFFER_LENGTH 256

/**
  * @brief  ��������
  * @param  desBuf:Ŀ�Ļ���
  * @param  srcBuf:Դ����
  * @param  cpCount:�踴�Ƶ����ݳ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
void ceBlueHc_cpData(uint8* desBuf, uint8* srcBuf, uint16 cpCount)
{
    uint16 i;
    for(i = 0; i < cpCount; i++)
    {
        desBuf[i] = srcBuf[i];
    }
    desBuf[i] = '\0';
}

/**
  * @brief  ��Uart�ж�ȡ���ݣ���endChar��β��β
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  buf:��������������õĻ���
  * @param  bufSize:�����С
  * @param  endChar:�����ݣ�ֱ���Դ��ַ�����β
  * @param  isCheckOK:�Ƿ���"OK\0"�ַ���
  * @param  checkOkStatus:���ڻ�ȡ�Ƿ����"OK\0"�ַ���0x00��ʾû�У�0x01��ʾ��
  * @param  outTimeMs:��ʱʱ��
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
uint16 ceBlueHc_readStringByEndChar(CeBlueHc* ceBlueHc, uint8* buf, uint16 bufSize, char* endChar, uint8 isCheckOK, uint8* checkOkStatus, uint16 outTimeMs)
{
    uint16 tickMs = 0;
    uint16 checkIndex = 0;
    uint16 bufIndex = 0;
    uint16 checkOKIndex = 0;
    char* tempOK = "OK\0";
    char temp[2];
    temp[1] = '\0';
    buf[0] = '\0';
    *checkOkStatus = 0x00;
    while(1)
    {
        if(ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)) <= 0)    //���û���յ���
        {
            if(tickMs >= outTimeMs)
            {
                buf[0] = '\0';
                return 0;
            }
            tickMs += 10;
            ceSystemOp.delayMs(10);
        }
        else   //������յ���
        {
            ceUartOp.readData(&(ceBlueHc->ceUart), (uint8*)(temp), 1);//��ȡһ���ֽ�
            //ceDebugOp.printf(temp);//���յ������ݴ����������ʱʹ��
            buf[bufIndex] = temp[0];
            bufIndex++;
            if(bufIndex == bufSize) //��ȡ�������ݳ����˽��ջ������󳤶ȣ���ʱӦ���ǽ��յ�������������Ӧ��ֱ�ӷ��ء�
            {
                buf[0] = '\0';
                return 0;
            }
            if(isCheckOK == 0x01)
            {
                if(temp[0] == tempOK[checkOKIndex])
                {
                    checkOKIndex++;
                    if(tempOK[checkOKIndex] == '\0')
                    {
                        *checkOkStatus = 0x01;
                        if(bufIndex > 5)
                        {
                            buf[bufIndex - ceStringOp.strlen(tempOK)] = '\0';
                            return bufIndex - ceStringOp.strlen(tempOK) + 1;
                        }
                        return 0;
                    }
                }
            }
            if(temp[0] == endChar[checkIndex])         //�Ƚ϶�ȡ�����ֽ�����Ҫ�Աȵ������Ƿ���ͬ
            {
                checkIndex++;                       //�����ͬ�����ñȽϵ���������1
                if(endChar[checkIndex] == '\0')     //�������1�󣬼���һ��Ҫ�Ƚϵ��ֽ��ǿգ�������Ƚ���ɣ����سɹ�
                {
                    buf[bufIndex - ceStringOp.strlen(endChar)] = '\0';
                    return bufIndex - ceStringOp.strlen(endChar) + 1;
                }
            }
            else                                    //�������ͬ������������Ϊ0���ٴ�0���¿�ʼ�Ƚ�
            {
                checkIndex = 0;
            }
        }
    }
}

/**
  * @brief  ��Uart�з����ݸ�ģ�飬���ȴ�ģ�鷵�ؽ��
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  sendMsg:Ҫ���ͳ�ȥ������
  * @param  recvMsg:�������յ�������,����ֻ����Ƿ����
  * @param  outTimeMs:��ʱʱ��
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
CE_STATUS ceBlueHc_sendDataAndCheck(CeBlueHc* ceBlueHc, char* sendMsg, char* recvMsg, uint16 outTimeMs)
{
    if(sendMsg == CE_NULL)
    {
        return CE_STATUS_FAILE;
    }
    else
    {
        uint32 tickMs = 0;
        uint16 checkIndex = 0;
        uint16 checkFIndex = 0;
        char* tempFalse = "FAIL\0";
        char temp[2];
        temp[1] = '\0';
        while(ceBlueHc->isLockRecvBuf == 0x01)
        {
            ceSystemOp.delayMs(1);
            tickMs++;
            if(tickMs >= 4000)
            {
                break;
            }
        }
        ceBlueHc->isLockRecvBuf = 0x01;
        if(sendMsg != CE_NULL)
        {
            ceUartOp.clearRecvBuf(&(ceBlueHc->ceUart));
            ceUartOp.sendData(&(ceBlueHc->ceUart), (uint8*)sendMsg, ceStringOp.strlen(sendMsg));
            ceSystemOp.delayMs(500);//Ҫô��Fifo�����⣬Ҫô��Uart�����⣬�������˲���ʱ���ᵼ���ղ�������
            //ceDebugOp.printf("Recv Count:%d \n",ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)));
        }
        while(1)
        {
            if(ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)) <= 0)    //���û���յ���
            {
                ceSystemOp.delayMs(10);
                tickMs += 10;
                if(tickMs >= outTimeMs)
                {
                    ceBlueHc->isLockRecvBuf = 0x00;
                    return CE_STATUS_OUT_TIME;
                }
            }
            else   //������յ���
            {
                ceUartOp.readData(&(ceBlueHc->ceUart), (uint8*)(temp), 1);//��ȡһ���ֽ�
                ceDebugOp.printf(temp);//���յ������ݴ����������ʱʹ��
                if(temp[0] == tempFalse[checkFIndex])
                {
                    checkFIndex++;
                    if(tempFalse[checkFIndex] == '\0')
                    {
                        return CE_STATUS_FAILE;
                    }
                }
                if(temp[0] == recvMsg[checkIndex])         //�Ƚ϶�ȡ�����ֽ�����Ҫ�Աȵ������Ƿ���ͬ
                {
                    checkIndex++;                       //�����ͬ�����ñȽϵ���������1
                    if(recvMsg[checkIndex] == '\0')     //�������1�󣬼���һ��Ҫ�Ƚϵ��ֽ��ǿգ�������Ƚ���ɣ����سɹ�
                    {
                        ceBlueHc->isLockRecvBuf = 0x00;
                        return CE_STATUS_SUCCESS;
                    }
                }
                else                                    //�������ͬ������������Ϊ0���ٴ�0���¿�ʼ�Ƚ�
                {
                    checkIndex = 0;
                }
            }
        }
    }
}

/**
  * @brief  CeBlueHcģ���鲨����
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @return ϵͳ״̬��
  */
CE_STATUS ceBlueHc_checkUartRate(CeBlueHc* ceBlueHc)
{
    CE_STATUS ceStatus;
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT\r\n", "OK", 2000);//AT
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT", ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  CeBlueHcģ���ʼ��
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  ceXX:CeBlueHcģ��ʹ�õ���Դ��
  * @return ϵͳ״̬��
  */
CE_STATUS ceBlueHc_initial(CeBlueHc* ceBlueHc, CE_RESOURCE ceUart, CE_RESOURCE ceGpio0,CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2)
{
    uint8 retry = 0;
    CE_STATUS ceStatus = CE_STATUS_FAILE;
    //Gpio0�������ģ�����µ磬0�ϵ�1���磻Gpio1����ATģʽ���ǹ���ģʽ��1ΪATģʽ�� Gpio2����������״̬��1δ���ӣ�0�ѽ������ӡ�
    ceBlueHc->ceGpio0.ceResource = ceGpio0;
    ceBlueHc->ceGpio0.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBlueHc->ceGpio0));

    ceBlueHc->ceGpio1.ceResource = ceGpio1;
    ceBlueHc->ceGpio1.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBlueHc->ceGpio1));

    ceBlueHc->ceGpio2.ceResource = ceGpio2;
    ceBlueHc->ceGpio2.ceGpioMode = CE_GPIO_MODE_IN_FLOATING;
    ceGpioOp.initial(&(ceBlueHc->ceGpio2));

    ceBlueHc->ceUart.ceResource = ceUart;
    ceBlueHc->ceUart.uartBaudRate = CE_UART_BAUD_RATE_38400;//CE_UART_BAUD_RATE_115200
    ceBlueHc->ceUart.uartParity = CE_UART_PARITY_NO;
    ceBlueHc->ceUart.uartStopBits = CE_UART_STOP_BITS_1;
    ceBlueHc->ceUart.uartWordLength = CE_UART_WORD_LENGTH_8B;
    ceBlueHc->ceUart.recvBufSize = CE_BLUE_HC_RECV_BUF_SIZE;
    ceBlueHc->ceUart.recvBuf = ceBlueHc->uartRecvBuf;
    ceBlueHc->ceUart.pAddPar = ceBlueHc;
    ceUartOp.initial(&(ceBlueHc->ceUart));
    ceUartOp.start(&(ceBlueHc->ceUart));

    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));//�Ƚ���AT״̬�����ģ�������Ƿ�����
    ceGpioOp.setBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(20);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));

    while(ceStatus != CE_STATUS_SUCCESS)
    {
        ceStatus = ceBlueHc_checkUartRate(ceBlueHc);
        retry++;
        if (retry > 10)
        {
            break;
        }
        ceSystemOp.delayMs(100);
    }
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));//Ĭ�Ͻ�����������ģʽ�����ڲ������ú����ڣ��Ž���ATģʽ
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));
    return ceStatus;
}

/**
  * @brief  CeBlueHcģ���������
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  ceBlueHcWorkMode:CeBlueHcģ��Ĺ�����ʽ������ģ��ʹ�ģ��
  * @param  devName:���豸������
  * @param  ceBlueHcDevType:��ģ�鹤������ģʽʱ�����Ҵ��豸ʱֻ���Ҵ����͵ģ���ģ�鹤���ڴ�ģʽʱ��Ϊ��ģ�������
  * @return ϵͳ״̬��
  */
CE_STATUS ceBlueHc_parmentConfig(CeBlueHc* ceBlueHc, CE_BLUE_HC_WORK_MODE ceBlueHcWorkMode, const char*  ceBlueHcDevType, const char* devName, const char* password)
{
    char sendBuf[64];
    CE_STATUS ceStatus;
    uint8 i=0;

    ceGpioOp.setBit(&(ceBlueHc->ceGpio1)); //ģ��������һ�µ磬����AT����
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));

    for(i=0;i<10;i++)//���ٽ����������ϸ�λ���ܳɹ�
    {
        ceDebugOp.sprintf(sendBuf, "AT+RESET\r\n");
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        if(ceStatus != CE_STATUS_SUCCESS) continue;
        else break;
    }



    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT+INIT\r\n", "OK", 2000);//��ʼ��SPP �淶�⣬ֻ�ܽ���һ�γ�ʼ������γ�ʼ������ERROR��17
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+INIT\r\n", ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;


    ceDebugOp.sprintf(sendBuf, "AT+NAME=%s\r\n", devName);
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 3000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    /*
    ceDebugOp.sprintf(sendBuf, "AT+RMAAD\r\n");//����������б���ɾ��������֤�豸
    retry = 0;
    do
    {
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 3000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        retry++;
    }
    while(ceStatus != CE_STATUS_SUCCESS && retry < 10);
    */


    ceDebugOp.sprintf(sendBuf, "AT+ROLE=%d\r\n", ceBlueHcWorkMode);
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+PSWD=%s\r\n", password);//����/��ѯ-�����
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 4000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;


   /* if(ceBlueHcWorkMode == CE_BLUE_HC_WORK_MODE_SLAVE)
    {
        return CE_STATUS_SUCCESS;
    }
    */

    ceDebugOp.sprintf(sendBuf, "AT+IAC=%s\r\n", "9e8b33");//���ò�ѯ���������������豸
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+CLASS=%s\r\n", ceBlueHcDevType);//���������豸����
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+INQM=0,%d,%d\r\n", CE_BLUE_HC_DEV_LIST_SIZE, CE_BLUE_HC_FIND_DEV_OUT_TIME);//��ѯģʽ����ѯһ����豸������CE_BLUE_HC_DEV_LIST_SIZE�������豸��Ӧ����ֹ��ѯ���趨��ʱΪCE_BLUE_HC_FIND_DEV_OUT_TIME
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 4000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    return ceStatus;
}

/**
  * @brief  �˳���������״̬��AT״̬����ģ�������ϵ磬��������������ģʽ
  * @param  ceBlueHc:CeBlueHc���Զ���
  */
void    ceBlueHc_outParmentConfig(CeBlueHc* ceBlueHc)
{
    ceSystemOp.delayMs(50);
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));     //ģ�鸴λ���������Զ������豸
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));
}

/**
  * @brief  ģʽ��������ģʽʱ��������Χ�п����ӵ�������Ϣ
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @return �������ӵ�������Ϣ����
  */
CeBlueHcDevInfo*  ceBlueHc_getCanConnectDevInfo(CeBlueHc* ceBlueHc)
{
    uint8 dataBuf[CE_BLUE_HC_BUFFER_LENGTH];
    char sendBuf[64];
    CE_STATUS ceStatus;
    uint16 recvDataCount;
    uint16 i;
    uint8 checkOkStatus = 0x00;
    for(i = 0; i < CE_BLUE_HC_DEV_LIST_SIZE; i++)
    {
        ceBlueHc->ceBlueHcDevInfoList[i].devAdress[0] = '\0';
        ceBlueHc->ceBlueHcDevInfoList[i].devName[0] = '\0';
    }
    ceBlueHc->ceBlueHcDevInfoFindDevCount = 0;

    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT+INQ\r\n", "+INQ:", CE_BLUE_HC_FIND_DEV_OUT_TIME * 1300);//
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+INQ\r\n", ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS)
    {
        ceBlueHc->ceBlueHcDevInfoFindDevCount = 0;
        return ceBlueHc->ceBlueHcDevInfoList;
    }

    while(1)
    {

        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "\r\n", 0x01, &checkOkStatus, (CE_BLUE_HC_DEV_LIST_SIZE * 4000));//ecn
        if (recvDataCount < 2)
        {
            break;
        }
        ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[ceBlueHc->ceBlueHcDevInfoFindDevCount].devAdress, dataBuf, 14);//��ַ12λ��Ȼ�󸽼�����:��һ��14��
        ceBlueHc->ceBlueHcDevInfoList[ceBlueHc->ceBlueHcDevInfoFindDevCount].devAdress[14] = ('\0');
        ceBlueHc->ceBlueHcDevInfoFindDevCount++;

        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "+INQ:", 0x01, &checkOkStatus, (CE_BLUE_HC_DEV_LIST_SIZE * 4000));//ecn
        if (recvDataCount < 4)
        {
            break;
        }
    }

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        uint8 j;
        for(j = 0; j < 32; j++)
        {
            if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == ':')
            {
                ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] = ',';
            }
            else if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == '\0')
            {
                break;
            }else if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == ',')
            {
                ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] = '\0';
            }
        }
    }

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        ceDebugOp.sprintf(sendBuf, "AT+RNAME? %s\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress);
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "+RNAME:", 5000);//
        #ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
        #endif
        if(ceStatus != CE_STATUS_SUCCESS)
        {
            ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, (uint8*)"Unknow device", ceStringOp.strlen("Unknow device"));
            continue;
        }
        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "OK", 0x01, &checkOkStatus, 2000);
        if(recvDataCount < 1)
        {
            ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, (uint8*)"Unknow device", ceStringOp.strlen("Unknow device"));
            continue;
        }
        ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, dataBuf, recvDataCount - (ceStringOp.strlen("\r\n") + 1));//�����������\r\n
        ceBlueHc->ceBlueHcDevInfoList[i].devName[recvDataCount - 2 + 1] = '\0';
        #ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("devName: %s\n", ceBlueHc->ceBlueHcDevInfoList[i].devName);
        #endif

    }
    return  ceBlueHc->ceBlueHcDevInfoList;
}

/**
  * @brief  ģʽ��������ģʽʱ��������Χ�п����ӵ������豸����
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @return ���ؿ����ӵ������豸����
  */
uint8 ceBlueHc_getCanConnectDevCount(CeBlueHc* ceBlueHc)
{
    ceBlueHc_getCanConnectDevInfo(ceBlueHc);
    return ceBlueHc->ceBlueHcDevInfoFindDevCount;
}

/**
  * @brief  ģʽ��������ģʽʱ������ָ���������Ƶ��豸�Ƿ���ڲ����ڿ�����״̬
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  devBlueName:��Ҫ���Ĵ��豸����
  * @return ����CE_STATUS_SUCCESS��ʾ�����ӣ� ����������ʾ��������
  */
CE_STATUS ceBlueHc_checkDevIsExist(CeBlueHc* ceBlueHc, const char* devBlueName)
{
    int i;
    ceBlueHc_getCanConnectDevInfo(ceBlueHc);
    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        //ceDebugOp.printf("devBlueName: %s devName: %s\n", devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName);
        if(ceStringOp.strcmp(devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName) == 0)
        {
            return CE_STATUS_SUCCESS;
        }
    }
    return CE_STATUS_FAILE;
}

/**
  * @brief  ģʽ��������ģʽʱ��ʹ���豸����������һ�����豸
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  devBlueName:��Ҫ���ӵĴ��豸����
  * @return ����CE_STATUS_SUCCESS��ʾ���ӳɹ���
  */
CE_STATUS ceBlueHc_connectDevByName(CeBlueHc* ceBlueHc, const char* devBlueName)
{
    char sendBuf[64];
    uint8 i;
    CE_STATUS ceStatus;

    ceBlueHc_getCanConnectDevInfo(ceBlueHc);

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        if(ceStringOp.strcmp(devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName) == 0)
        {
            //����ԣ�Ҳ����ֱ������
            ceDebugOp.sprintf(sendBuf, "AT+PAIR=%s,%d\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress, 20);
            ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 30000);
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
            if(ceStatus != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }

            ceDebugOp.sprintf(sendBuf, "AT+LINK=%s\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress);
            ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 20000);//���δ���ֱ�����ӣ���ʱʱ��Ӧ���ó�һЩ��Ӧ����Ҫ�ȴ����豸����ȷ����
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
            if(ceStatus != CE_STATUS_SUCCESS)
            {
                //����յ���Ӧ�����룬Ӧ�ý���һЩ����
                return ceStatus;
            }
        }
    }
    return ceStatus;
}

/**
  * @brief  ��ȡģ�������״̬
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @return ����0x00:ģ�鴦��δ���״̬������0x01:ģ�鴦����Գɹ�״̬�����Խ������ݴ���
  */
uint8 getConnectStatus(CeBlueHc* ceBlueHc)
{
    /*
    //��ȡ״̬
    ceDebugOp.sprintf(sendBuf, "AT+STATE?\r\n");
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 20000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if(ceStatus == CE_STATUS_SUCCESS)
    {
        return CE_STATUS_SUCCESS;
    }
    return CE_STATUS_FAILE;*/
    return ceGpioOp.getBit(&(ceBlueHc->ceGpio2));//���б�Ҫ�����Բ�������ķ�ʽ
}

/**
  * @brief  ��������
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  dataInBuf:��Ҫ���͵����ݻ���
  * @param  sendCount:��Ҫ����
  * @return ʵ�ʷ�����ɵ����ݳ���
  */
void ceBlueHc_sendData(CeBlueHc* ceBlueHc, uint8* dataInBuf, uint16 sendCount)
{
    uint16 timeOutMs = 0;
    while(ceBlueHc->isLockRecvBuf == 0x01)
    {
        ceSystemOp.delayMs(1);
        timeOutMs++;
        if(timeOutMs >= 2000)
        {
            break;
        }
    }
    ceBlueHc->isLockRecvBuf = 0x01;
    ceUartOp.sendData(&(ceBlueHc->ceUart), dataInBuf, sendCount);
    ceBlueHc->isLockRecvBuf = 0x00;
}

/**
  * @brief  ��ȡ���ջ����еĿɶ�ȡ�����ݳ���
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @return ���ջ����еĿɶ�ȡ����������
  */
uint16 ceBlueHc_getRecvDataCount(CeBlueHc* ceBlueHc)
{
    return ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart));
}

/**
  * @brief  �ӽ��ջ����ж�ȡ����
  * @param  ceBlueHc:CeBlueHc���Զ���
  * @param  dataOutBuf:��ȡ���ݴ�ŵĻ���
  * @param  readCount:��Ҫ��ȡ�����ݳ���
  * @return ʵ�ʶ�ȡ�������ݳ���
  */
uint16 ceBlueHc_readData(CeBlueHc* ceBlueHc, uint8* dataOutBuf, uint16 readCount)
{
    return ceUartOp.readData(&(ceBlueHc->ceUart), dataOutBuf, readCount);
}

/**
  * @brief  CeBlueHcģ�����������
  */
const CeBlueHcOpBase ceBlueHcOp = {ceBlueHc_initial, ceBlueHc_parmentConfig, ceBlueHc_outParmentConfig,ceBlueHc_getCanConnectDevInfo, ceBlueHc_getCanConnectDevCount,
                                   ceBlueHc_checkDevIsExist, ceBlueHc_connectDevByName, getConnectStatus,
                                   ceBlueHc_sendData, ceBlueHc_getRecvDataCount, ceBlueHc_readData
                                  };

#ifdef __cplusplus
}
#endif //__cplusplus
