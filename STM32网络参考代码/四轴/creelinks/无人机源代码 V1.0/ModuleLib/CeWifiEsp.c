/**
  ******************************************************************************
  * @file    CeWifiEsp.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeWifiEspģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include"CeWifiEsp.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef CE_WIFI_ESP_UT
#define CE_WIFI_ESP_CONNECT_MUL         0                               /*!< ESP8266����������0Ϊ�����ӣ�1Ϊ������*/
#else
#define CE_WIFI_ESP_CONNECT_MUL         1                               /*!< ESP8266����������0Ϊ�����ӣ�1Ϊ������*/
#endif

/**
  * @brief  ��������
  * @param  desBuf:Ŀ�Ļ���
  * @param  srcBuf:Դ����
  * @param  cpCount:�踴�Ƶ����ݳ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
void ceWifiEsp_cpData(uint8* desBuf, uint8* srcBuf, uint16 cpCount)
{
    uint16 i;
    for (i = 0; i < cpCount; i++)
    {
        desBuf[i] = srcBuf[i];
    }
}

/**
  * @brief  ��Uart�ж�ȡ���ݣ���endChar��β��β
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  buf:��������������õĻ���
  * @param  bufSize:�����С
  * @param  endChar:�����ݣ�ֱ���Դ��ַ�����β
  * @param  outTimeMs:��ʱʱ��
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
uint16 ceWifiEsp_readStringByEndChar(CeWifiEsp* ceWifiEsp, uint8* buf,uint16 bufSize, char* endChar, uint16 outTimeMs)
{
    uint16 tickMs = 0;
    uint16 checkIndex = 0;
    uint16 bufIndex = 0;
    char temp[2];
    temp[1] = '\0';
    buf[0] = '\0';

    //ceDebugOp.printf("endChar:%s\n ,",endChar);
    
    while (1)
    {
        if (ceUartOp.getRecvDataCount(&(ceWifiEsp->ceUart)) <= 0)   //���û���յ���
        {
            if (tickMs >= outTimeMs)
            {
                buf[0] = '\0';
                return 0;
            }
            tickMs += 10;
            ceSystemOp.delayMs(10);
        }
        else   //������յ���
        {
            ceUartOp.readData(&(ceWifiEsp->ceUart), (uint8*)(temp), 1);//��ȡһ���ֽ�

            if (ceWifiEsp->isServerMode != 0x00 || ceWifiEsp->isInUTSend == 0x01)
            {
//ceDebugOp.printf("%s,",temp[0]);//���յ������ݴ����������ʱʹ��
            }
                        

            buf[bufIndex] = temp[0];
            bufIndex++;
            if (bufIndex == bufSize)//��ȡ�������ݳ����˽��ջ������󳤶ȣ���ʱӦ���ǽ��յ�������������Ӧ��ֱ�ӷ��ء�
            {
                buf[0] = '\0';            
//ceDebugOp.printf("ddd\n");                            
                return 0;
            }
            if (temp[0] == endChar[checkIndex])        //�Ƚ϶�ȡ�����ֽ�����Ҫ�Աȵ������Ƿ���ͬ
            {
                checkIndex++;                       //�����ͬ�����ñȽϵ���������1
                if (endChar[checkIndex] == '\0')    //�������1�󣬼���һ��Ҫ�Ƚϵ��ֽ��ǿգ�������Ƚ���ɣ����سɹ�
                {
                    buf[bufIndex - ceStringOp.strlen(endChar)] = '\0';
                    return bufIndex - ceStringOp.strlen(endChar)+1;
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
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  sendMsg:Ҫ���ͳ�ȥ������
  * @param  recvMsg:�������յ�������,����ֻ����Ƿ����
  * @param  outTimeMs:��ʱʱ��
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_sendDataAndCheck(CeWifiEsp* ceWifiEsp, char* sendMsg, char* recvMsg, uint16 outTimeMs)
{
    if (sendMsg == CE_NULL && ceWifiEsp->isInUTSend == 0x01)
    {
        return CE_STATUS_SUCCESS;
    }
    else
    {
        uint16 tickMs = 0;
        uint16 checkIndex = 0;
        uint16 checkEIndex = 0;
        uint16 timeOutMs = 0;
        char temp[2];
        char* tempError = "ERROR";
        temp[1] = '\0';


        while (ceWifiEsp->isLockRecvBuf == 0x01)
        {
            ceSystemOp.delayMs(1);
            timeOutMs++;
            if (timeOutMs >= 2000)
            {
                break;
            }
        }
        ceWifiEsp->isLockRecvBuf = 0x01;

        if (sendMsg != CE_NULL)
        {
            ceUartOp.clearRecvBuf(&(ceWifiEsp->ceUart));
            ceUartOp.sendData(&(ceWifiEsp->ceUart), (uint8*)sendMsg, ceStringOp.strlen(sendMsg));
        }
        while (1)
        {
            if (ceUartOp.getRecvDataCount(&(ceWifiEsp->ceUart)) <= 0)   //���û���յ���
            {
                ceSystemOp.delayMs(10);
                tickMs += 10;
                if (tickMs >= outTimeMs)
                {
                    ceWifiEsp->isLockRecvBuf = 0x00;
                    return CE_STATUS_OUT_TIME;
                }
            }
            else   //������յ���
            {
                ceUartOp.readData(&(ceWifiEsp->ceUart), (uint8*)(temp), 1);//��ȡһ���ֽ�

                //ceDebugOp.printf(temp);//���յ������ݴ����������ʱʹ��

                if (temp[0] == tempError[checkEIndex])
                {
                    checkEIndex++;
                    if (tempError[checkEIndex] == '\0')
                    {
                        return CE_STATUS_FAILE;
                    }
                }

                if (temp[0] == recvMsg[checkIndex])        //�Ƚ϶�ȡ�����ֽ�����Ҫ�Աȵ������Ƿ���ͬ
                {
                    checkIndex++;                       //�����ͬ�����ñȽϵ���������1
                    if (recvMsg[checkIndex] == '\0')    //�������1�󣬼���һ��Ҫ�Ƚϵ��ֽ��ǿգ�������Ƚ���ɣ����سɹ�
                    {
                        ceWifiEsp->isLockRecvBuf = 0x00;
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
  * @brief  ����ģ�鳬ʱʱ��
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  outTime:��ʱʱ��
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_atCIpSto(CeWifiEsp* ceWifiEsp,uint16 outTime)
{
    char sendBuf[30];
    CE_STATUS ceStatus;
    ceDebugOp.sprintf(sendBuf, "AT+CIPSTO=%d\r\n", outTime);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "\r\nOK\r\n",500);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s, run result:%s\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    return ceStatus;
}
/**
  * @brief  ��͸��ģʽ�£���������ǰ������
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  id:linkID��
  * @param  dataSize:���ݳ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_atCIpSend(CeWifiEsp* ceWifiEsp,uint8 id, uint16 dataSize)
{
    if ((ceWifiEsp->isInUTSend == 0x01 || CE_WIFI_ESP_CONNECT_MUL == 0 )&& ceWifiEsp->isServerMode != 0x01)
    {
        return CE_STATUS_SUCCESS;
    }
    else
    {
        char sendBuf[30];
        CE_STATUS ceStatus;
        if (CE_WIFI_ESP_CONNECT_MUL == 0)
        {
                      if(ceWifiEsp->isServerMode == 0x01)
                            ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d,%d\r\n", id, dataSize);
             else
                ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d\r\n", dataSize);
        }
        else
        {
            ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d,%d\r\n", id, dataSize);                    
        }
        ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, ">",1000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        return ceStatus;
    }
}

/**
  * @brief  ��Uart������
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  dataBuf:Ҫ���͵����ݻ���
  * @param  dataSize:Ҫ���͵����ݳ���
  */
void ceWifiEsp_sendUartData(CeWifiEsp* ceWifiEsp, uint8* dataBuf, uint16 dataSize)
{
    uint16 timeOutMs = 0;
    while (ceWifiEsp->isLockRecvBuf == 0x01)
    {
        ceSystemOp.delayMs(1);
        timeOutMs++;
        if (timeOutMs >= 2000)
        {
            break;
        }
    }
    ceWifiEsp->isLockRecvBuf = 0x01;
    ceUartOp.sendData(&(ceWifiEsp->ceUart), dataBuf, dataSize);
    ceWifiEsp->isLockRecvBuf = 0x00;
}

/**
  * @brief  ����Task���������ڼ��������ݣ��������û��ص�
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  pAddPar:CeWifiEspָ��
  */
void ceWifiEsp_taskCallBack(void* pAddPar)
{
    CeWifiEsp* ceWifiEsp = (CeWifiEsp*)(pAddPar);
    if (ceWifiEsp->isLockRecvBuf == 0x01)
        return;

    if ((ceWifiEsp->isInUTSend == 0x00 &&ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) <= 9) || (ceWifiEsp->isInUTSend == 0x01 &&ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) == 0))
        return;

    else
    {
        uint16 recvDataCount; 
        uint8 linkID = 0;
        if (ceWifiEsp->isInUTSend == 0x01)
        {
            recvDataCount = ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart);
        }
        else
        {
            uint8 dataBuf[30];
            uint16 outTimeMs = 0;
            recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, (uint8*)dataBuf, 30, "+IPD,",500);//
            if (recvDataCount == 0)         //���û�ж������ݣ���ʱuart����������Ѷ��꣬����Ҳ������ʱ��
                return;

            if(ceWifiEsp->isServerMode == 1 || CE_WIFI_ESP_CONNECT_MUL == 1)//������ģʽ��һ��Ϊ������
            {
                recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",",500);//��ȡ���ĸ��ͻ������ӷ�����������
                if (recvDataCount == 0 || recvDataCount > 2 || (dataBuf[0] - 0x30) > 4)
                    return;

                linkID = dataBuf[0] - 0x30;
            }else if(CE_WIFI_ESP_CONNECT_MUL == 0)
                linkID = 0;                                            

            recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ":",500);//��ȡ���ݳ���
            if (recvDataCount == 0 || recvDataCount > 6)
                return;


            recvDataCount = ceStringOp.atoi((char*)dataBuf);
            if (recvDataCount <= 0 || recvDataCount >= 1500)//�����������ݳ����Ƿ�����Ҫ��
                return;

            while (ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) < recvDataCount)//Uart���ջ����е����ݣ���������Ҫ��ȡ�����ݳ���
            {
                ceSystemOp.delayMs(1);
                outTimeMs++;
                if (outTimeMs > 2000)
                    return;
            }
        }
        if (recvDataCount != ceUartOp.readData(&(ceWifiEsp->ceUart), ceWifiEsp->recvData, recvDataCount))
            return;

        ceWifiEsp->recvData[recvDataCount] = '\0';

        if (ceWifiEsp->isServerMode == 0x00)
        {
            if (ceWifiEsp->callBackClientRecv[linkID] == CE_NULL)
                return;

            ceWifiEsp->callBackClientRecv[linkID](ceWifiEsp->recvData, recvDataCount);//����Ļص������⣬����Ǻ���ָ�����������
        }
        else
        {
            if (ceWifiEsp->callBackServerRecv == CE_NULL)
            {
                return;
            }
            ceWifiEsp->callBackServerRecv(linkID, ceWifiEsp->recvData, recvDataCount);
        }
    }
}

/**
  * @brief  ʹ��һ��UART��Դ����ʼ��CeWifiEspģ��
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  ceUart:��ģ��ʹ�õ���Uart��Դ�ӿ�
  * @param  ceWifiEspMode:��ģ��Ĺ�����ʽ����AP��STA��ʽ
  *         ��APģʽ��ģ����Ϊ�ȵ㣬�����豸����ģ�飩����ģ������
  *         ��STA��ģ����Ϊ���豸�����������Ѵ��ڵ��ȵ㣬�����·�������ֻ��Ͻ������ȵ�ȣ�����ģ������
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_initial(CeWifiEsp* ceWifiEsp, CE_RESOURCE ceUart)
{
   
    ceWifiEsp->isLockRecvBuf = 0x00;
    ceWifiEsp->isInUTSend = 0x00;
    ceWifiEsp->isServerMode = 0x00;
    ceWifiEsp->linkInfoCount = 0;
    ceWifiEsp->apLinkDeviceCount = 0;
    ceWifiEsp->staCanConnectwifiCount = 0;
    ceWifiEsp->callBackServerRecv = CE_NULL;
    ceWifiEsp->callBackClientRecv[0] = CE_NULL;
    ceWifiEsp->callBackClientRecv[1] = CE_NULL;
    ceWifiEsp->callBackClientRecv[2] = CE_NULL;
    ceWifiEsp->callBackClientRecv[3] = CE_NULL;
    ceWifiEsp->callBackClientRecv[4] = CE_NULL;

    ceWifiEsp->ceUart.ceResource = ceUart;
    ceWifiEsp->ceUart.uartBaudRate = CE_UART_BAUD_RATE_115200;
    ceWifiEsp->ceUart.uartParity = CE_UART_PARITY_NO;
    ceWifiEsp->ceUart.uartStopBits = CE_UART_STOP_BITS_1;
    ceWifiEsp->ceUart.uartWordLength = CE_UART_WORD_LENGTH_8B;
    ceWifiEsp->ceUart.recvBufSize = CE_WIFI_ESP_RECV_BUF_SIZE;
    ceWifiEsp->ceUart.recvBuf = ceWifiEsp->uartRecvBuf;
    ceWifiEsp->ceUart.pAddPar = ceWifiEsp;
    ceUartOp.initial(&(ceWifiEsp->ceUart));
    ceUartOp.start(&(ceWifiEsp->ceUart));

    ceWifiEsp->ceTask.ID = ceUart;
    ceWifiEsp->ceTask.callBack = ceWifiEsp_taskCallBack;
    ceWifiEsp->ceTask.pAddPar = ceWifiEsp;
#ifdef CE_USE_RTOS
    ceWifiEsp->ceTask.isNewThread = 0x01;
    ceWifiEsp->ceTaskPriority = CE_TASK_PRIORITY_H;
#endif
    ceTaskOp.registerTask(&(ceWifiEsp->ceTask));
    ceTaskOp.start(&(ceWifiEsp->ceTask));

    return CE_STATUS_SUCCESS;
}


CE_STATUS   ceWifiEsp_setWorkMode(CeWifiEsp* ceWifiEsp,CE_WIFI_ESP_MODE ceWifiEspMode)
{
    char sendBuf[128];
    CE_STATUS ceStatus;

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT\r\n", "OK", 2000);       //AT+RST
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT",ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEspOp.stopUTSendOnClient(ceWifiEsp);//�������ATָ��û�л�Ӧ��������ǵ�ǰ������͸��ģʽ�£����ȳ����˳�͸��ģʽ
        ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT\r\n", "OK", 2000);       //AT+RST
        if(ceStatus != CE_STATUS_SUCCESS)
            return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CWMODE=%d\r\n", ceWifiEspMode);                     //AT+CWMODE=? ������ʽ��1��STA�� 2��AP�� 3��STA+AP
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+RST\r\n", "ready", 8000);       //AT+RST
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT+RST",ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CIPMUX=%d\r\n", CE_WIFI_ESP_CONNECT_MUL);           //AT+CIPMUX=? �Ƿ������0�������ӣ� 1��������
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }
    return ceStatus;
}


/**
  * @brief  ����һ��AP�ȵ�
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  ip:��AP�ȵ��IP��ַ���������ӵ���ģ����豸��IP�ڴ˻���������
  * @param  ssid:��AP�ȵ��Wifi����
  * @param  channelNumber:Wifi�����ŵ�����Χ1��13����ʼ��ʱ����Χ������ֵ���ɡ�
  * @param  ceWifiEspEcn:��AP�ȵ㴴����Wifi���ܷ�ʽ������CE_WIFI_ESP_ECN_WPA2_PSK
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_createWifi(CeWifiEsp* ceWifiEsp, const char* ip, const char* ssid, const char* passWord,uint8 channelNumber,CE_WIFI_ESP_ECN ceWifiEspEcn)
{
    char sendBuf[128];
    CE_STATUS ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+CWSAP=\"%s\",\"%s\",%d,%d\r\n", ssid, passWord, channelNumber, ceWifiEspEcn);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CIPAP=\"%s\"\r\n", ip);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    return ceStatus;
}

/**
  * @brief  AP������ʽ�£�������AP�ݵ����ӵĴ��豸��IP��MAC��ַ����
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ���豸��Ϣ����
  */
CeWifiEspAPLinkDevInfo* ceWifiEsp_getConnectedDeviceList(CeWifiEsp* ceWifiEsp)
{
    uint8 dataBuf[30];
    uint8 i;
    uint16 recvDataCount;
    CE_STATUS ceStatus;

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        ceWifiEsp->apLinkDevInfoList[i].ip[0] = '\0';
        ceWifiEsp->apLinkDevInfoList[i].mac[0] = '\0';
    }

#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp try to find connect device...\n");
#endif

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CWLIF\r\n", "AT+CWLIF\r\r\n", 1000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CWLIF\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEsp->apLinkDeviceCount = 0;
        return ceWifiEsp->apLinkDevInfoList;
    }

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",",2000);//��ȡip
        if (recvDataCount == 0 || recvDataCount > 16)
        {
            ceWifiEsp->apLinkDeviceCount = i;
            return  ceWifiEsp->apLinkDevInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->apLinkDevInfoList[i].ip), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\r\n", 2000);//��ȡMAC��ַ
        if (recvDataCount == 0 || recvDataCount > 19)
        {
            ceWifiEsp->apLinkDevInfoList[i].ip[0] = '\0';
            ceWifiEsp->apLinkDeviceCount = i;
            return  ceWifiEsp->apLinkDevInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->apLinkDevInfoList[i].mac), dataBuf, recvDataCount);

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find connect device on AP Mode:\nIP:%s    MAC:%s\n", ceWifiEsp->apLinkDevInfoList[i].ip, ceWifiEsp->apLinkDevInfoList[i].mac);
#endif
    }
    return ceWifiEsp->apLinkDevInfoList;
}

/**
  * @brief  AP������ʽ�£�������AP�ݵ����ӵĴ��豸���������������ж������豸���ӵ���Wifi֮��
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ���豸������
  */
uint8 ceWifiEsp_getConnectedDeviceCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getConnectedDeviceList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find connect device count: %d\n", ceWifiEsp->apLinkDeviceCount);
#endif
    return ceWifiEsp->apLinkDeviceCount;
}

/**
  * @brief  STA������ʽ�£�������Χ�����п����ӵ�Wifi�ź�
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ����Wifi������ָ��
  */
CeWifiEspCanConnectWifiInfo*  ceWifiEsp_getCanConnectWifiList(CeWifiEsp* ceWifiEsp)
{
    uint16 i;
    uint8 dataBuf[50];
    uint16 recvDataCount;
    CE_STATUS ceStatus;
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)//����������е�����
    {
        ceWifiEsp->staCanConnectWifiList[i].ssid[0] = '\0';
        ceWifiEsp->staCanConnectWifiList[i].mac[0] = '\0';
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CWLAP\r\n", "AT+CWLAP", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CWLAP\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEsp->staCanConnectwifiCount = 0;
        return ceWifiEsp->staCanConnectWifiList;
    }

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)//
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "+CWLAP:(", 8000);//���ҿ���wifi�ʱ�䣬���������Щ��ʱʱ��
        if (recvDataCount <= 0)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",\"", 2000);//ecn
        if (recvDataCount != 2 || dataBuf[0] - 0x30 > 5)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn = (CE_WIFI_ESP_ECN)(dataBuf[0] - 0x30);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "\",", 2000);//ssid
        if (recvDataCount <= 1)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->staCanConnectWifiList[i].ssid), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",\"", 2000);//signal
        if (recvDataCount <= 1 || recvDataCount >= 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].signal = ceStringOp.atoi((char*)dataBuf);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "\",", 2000);//MAC
        if (recvDataCount != 18)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->staCanConnectWifiList[i].mac), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",", 2000);//other1
        if (recvDataCount <= 0 || recvDataCount > 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].other1 = ceStringOp.atoi((char*)dataBuf);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ")", 2000);//other2
        if (recvDataCount <= 0 || recvDataCount > 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].other2 = ceStringOp.atoi((char*)dataBuf);

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find can connect wifi on STA Mode:\n ECN:%d, SSID:%s, Signal:%d, MAC:%s, O1:%d, O2:%d\n",
            ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn, ceWifiEsp->staCanConnectWifiList[i].ssid,
            ceWifiEsp->staCanConnectWifiList[i].signal,ceWifiEsp->staCanConnectWifiList[i].mac,
            ceWifiEsp->staCanConnectWifiList[i].other1, ceWifiEsp->staCanConnectWifiList[i].other2);
#endif
    }
    return ceWifiEsp->staCanConnectWifiList;
}

/**
  * @brief  STA������ʽ�£�������Χ�����п����ӵ�Wifi�ź�����
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ����Wifi������
  */
uint8  ceWifiEsp_getCanConnectWifiCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getCanConnectWifiList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find can connect wifi count: %d\n", ceWifiEsp->staCanConnectwifiCount);
#endif
    return ceWifiEsp->staCanConnectwifiCount;
}

/**
  * @brief  STA������ʽ�£�������Χ�������Ƿ��и�����ssid��Wifi�ź�
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_checkCanConnectSsidIsExist(CeWifiEsp* ceWifiEsp, const char* ssid)
{
    uint8  i;
    CeWifiEspCanConnectWifiInfo* canConnectWifiInfo = ceWifiEsp_getCanConnectWifiList(ceWifiEsp);
    if (ceWifiEsp->staCanConnectwifiCount == 0)
    {
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp cannot find wifi by name: %s\n", ssid);
#endif
        return CE_STATUS_FAILE;
    }
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        if (ceStringOp.strcmp(ceWifiEsp->staCanConnectWifiList[i].ssid, ssid) == 0)
        {
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("CeWifiEsp has find wifi by name: %s\n ECN:%d, SSID:%s, Signal:%d, MAC:%s, O1:%d, O2:%d\n", ssid,
                ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn, ceWifiEsp->staCanConnectWifiList[i].ssid,
                ceWifiEsp->staCanConnectWifiList[i].signal, ceWifiEsp->staCanConnectWifiList[i].mac,
                ceWifiEsp->staCanConnectWifiList[i].other1, ceWifiEsp->staCanConnectWifiList[i].other2);
#endif
            return CE_STATUS_SUCCESS;
        }
    }
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp cannot find wifi by name: %s\n", ssid);
#endif
    return CE_STATUS_FAILE;
}

/**
  * @brief  STA������ʽ������һ���Ѿ����ڵ��ȵ㣬����Ϊ�ȵ��SSID��PWD
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  ssid:��Ҫ���ӵ�Wifi����
  * @param  passWord:Wifi����
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_connectWifi(CeWifiEsp* ceWifiEsp, const char* ssid, const char* passWord)
{
    CE_STATUS ceStatus;
    char sendBuf[100];

    ceDebugOp.sprintf(sendBuf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, passWord);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 12000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif

#ifdef CE_WIFI_ESP_UT
    ceDebugOp.sprintf(sendBuf, "AT+CIPMODE=%d\r\n", 1);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s Run result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif

#endif

    return ceStatus;
}

/**
  * @brief  STA������ʽ�£����ӵ��ȵ���IP��ע���ʱ���õ�IPӦ���ȵ�����ID��ƥ�䡣
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  ip:��Ҫ�趨��IP��ַ
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_setStaIp(CeWifiEsp* ceWifiEsp, const char* ip)
{
    char sendBuf[64];
    CE_STATUS ceStatus;
    ceDebugOp.sprintf(sendBuf, "AT+CIPAP=\"%s\"\r\n", ip);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  ����һ�������������������뷢�����ݣ�AP��STAģʽ���ɴ���
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  serVerPortNum:�贴���ķ������˿ں�
  * @param  linkID:�����Ǵ����˷���������˷��������ֻ����5���ͻ����������ӣ�linkIDָ�ľ�����5���ͻ��˵ı�ţ����������������Ŀͻ���Ϊ0���ص���LinkID��ʾ���������յ������ݣ����������ĸ��ͻ��˷�������
  * @param  callBackServerRecv:���������յ����ݺ�Ļص�
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS    ceWifiEsp_createServer(CeWifiEsp* ceWifiEsp,uint16 serverPortNum, CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount))
{
    char sendBuf[30];
    CE_STATUS ceStatus;
    ceWifiEsp->isServerMode = 0x01;
    ceWifiEsp->callBackServerRecv = callBackServerRecv;
    
      ceDebugOp.sprintf(sendBuf, "AT+CIPMUX=%d\r\n", 1);           //AT+CIPMUX=? �Ƿ������0�������ӣ� 1��������
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }
    
    
    ceDebugOp.sprintf(sendBuf, "AT+CIPSERVER=%d,%d\r\n", 1, serverPortNum);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  ����һ���Ѵ��ڵķ����������������뷢�����ݣ�AP��STAģʽ���ɴ���
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  linkID:ESP8266���Դ���5���ͻ��ˣ����0��4���ֱ����Ӹ��Եķ�������LinkIDָʹ���ĸ��ͻ��˱�ź����ӷ�����
  * @param  serverIP:��Ҫ���ӵķ�������IP
  * @param  serVerPortNum:��Ҫ���ӵķ������Ķ˿�
  * @param ceWifiEspSocketMode:���ӷ�ʽ��TCP��UDP
  * @param callBackClientRecv:�˿ͻ��˽��յ����ݺ�Ļص�
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_connectServer(CeWifiEsp* ceWifiEsp,uint8 linkID, const char* serverIP,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8* recvBuf, uint16 recvCount))
{

    char sendBuf[50];
    CE_STATUS ceStatus;    
    
    ceWifiEsp->isServerMode = 0x00;
    ceWifiEsp->callBackClientRecv[linkID] = callBackServerRecv;

    if (CE_WIFI_ESP_CONNECT_MUL == 0)
        ceDebugOp.sprintf(sendBuf, "AT+CIPSTART=\"%s\",\"%s\",%d\r\n", ((ceWifiEspSocketMode == CE_WIFI_ESP_SOCKET_MODE_TCP) ? "TCP" : "UDP"), serverIP, serVerPortNum);
    else
        ceDebugOp.sprintf(sendBuf, "AT+CIPSTART=%d,\"%s\",\"%s\",%d\r\n", linkID, ((ceWifiEspSocketMode == CE_WIFI_ESP_SOCKET_MODE_TCP) ? "TCP" : "UDP"), serverIP, serVerPortNum);

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);

#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  ��ñ�ģ���������������б�����������ģʽ�£���˷��������ӵĿͻ����������ͻ���ģʽ�£��ж���ٿͻ������ڽ����������������ӣ�����5����
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return SOCKET��������
  */
CeWifiEspLinkInfo* ceWifiEsp_getLinkList(CeWifiEsp* ceWifiEsp)
{
    uint16 i;
    uint8 dataBuf[30];
    uint16 recvDataCount;
    CE_STATUS ceStatus;
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        ceWifiEsp->linkInfoList[i].ip[0] = '\0';
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CIPSTATUS\r\n", "+CIPSTATUS:", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CIPSTATUS\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp cannot find link on server or client mode.\n");
#endif
        ceWifiEsp->linkInfoCount = 0;
        return ceWifiEsp->linkInfoList;
    }
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",\"", 2000);//���Ӻţ�0��4
        if (recvDataCount == 0 || recvDataCount > 2 || (dataBuf[0] - 0x30) > 4)
        {
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].linkID = (dataBuf[0] - 0x30);


        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\",\"", 2000);//��ȡ���ӷ�ʽ��TCP��UDP
        if (recvDataCount == 0 || recvDataCount > 4)
        {
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        if (dataBuf[0] == 'T' && dataBuf[1] == 'C' && dataBuf[2] == 'P')
        {
            ceWifiEsp->linkInfoList[i].socketMode = CE_WIFI_ESP_SOCKET_MODE_TCP;
        }
        else if (dataBuf[0] == 'U' && dataBuf[1] == 'D' && dataBuf[2] == 'P')
        {
            ceWifiEsp->linkInfoList[i].socketMode = CE_WIFI_ESP_SOCKET_MODE_UDP;
        }
        else
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\",", 2000);//��ȡIP
        if (recvDataCount <7 || recvDataCount > 20)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->linkInfoList[i].ip), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",", 2000);//��ȡ�˿ں�
        if (recvDataCount == 0 || recvDataCount > 6)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].port = (uint16)(ceStringOp.atoi((char*)dataBuf));


        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\r\n", 2000);//��ȡ��ģ������Ϊ�����������ǿͻ������ӣ�1��Ϊ��������0��Ϊ�ͻ���
        if (recvDataCount == 0 || recvDataCount > 2)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].moduleLinkType = (uint16)(ceStringOp.atoi((char*)dataBuf));

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find one link:\n LinkID:%d, Socket mode:%s, IP:%s, Port:%d, LinkProperty:Module as %s\n",
            ceWifiEsp->linkInfoList[i].linkID, ((ceWifiEsp->linkInfoList[i].socketMode == CE_WIFI_ESP_SOCKET_MODE_TCP)?"TCP":"UDP"),
            ceWifiEsp->linkInfoList[i].ip, ceWifiEsp->linkInfoList[i].port,((ceWifiEsp->linkInfoList[i].moduleLinkType == 1)?"Server":"Client"));
#endif
    }
    return ceWifiEsp->linkInfoList;
}

/**
  * @brief  ��ñ�ģ�����������������������������жϣ�������������ʽ�£����޿ͻ������ӡ��ͻ��˷�ʽ�£��Ƿ��������ӵ���������
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return SOCKET��������
  */
uint8 ceWifiEsp_getLinkCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getLinkList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find link count: %d\n", ceWifiEsp->linkInfoCount);
#endif
    return ceWifiEsp->linkInfoCount;
}

/**
  * @brief  ����͸����ֻ��ģ���ǵ����ӣ���Ϊ�ͻ��˲ſ����д˹���
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_startUTSendOnClient(CeWifiEsp* ceWifiEsp)
{
    CE_STATUS ceStatus;
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CIPSEND\r\n", ">",8000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CIPSEND\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    ceWifiEsp->isInUTSend = 0x01;
    return ceStatus;
}

/**
  * @brief  ֹͣ͸����ֻ��ģ���ǵ����ӣ���Ϊ�ͻ��˲ſ����д˹���
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_stopUTSendOnClient(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_sendDataAndCheck(ceWifiEsp, "+++", CE_NULL,8000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\n\n", "+++");
#endif
    ceWifiEsp->isInUTSend = 0x00;
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  ��һ�����Ӻ�linkID�������ݡ�
  * @param  ceWifiEsp:CeWifiEsp���Զ���
  * @param  linkID:������ģʽ�£������Ӻ�ָ�����������ӵĿͻ�����ţ��ͻ���ģʽ�£������Ӻ�ָʹ���ĸ��ͻ��ˣ����0��4��
  * @param  dataBuf:��Ҫ���͵����ݻ���
  * @param  dataSize:��Ҫ���͵����ݳ���
  * @return ϵͳ״̬�룬���ܵ�ֵ��CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_sendData(CeWifiEsp* ceWifiEsp, uint8 linkID, uint8* dataBuf, uint16 dataSize)
{
    if (dataSize <= 1400)//socket һ�������1480���ֽڣ������1400���ֽ�
    {
        if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, dataSize) != CE_STATUS_SUCCESS)
        {
            return CE_STATUS_FAILE;
        }
        ceWifiEsp_sendUartData(ceWifiEsp, dataBuf, dataSize);
        if (ceWifiEsp->isInUTSend)
            return CE_STATUS_SUCCESS;
        else
            return ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000);
    }
    else
    {
        uint8 loop = dataSize / 1400;
        uint16 lastData = dataSize % 1400;
        uint8 i;
        for (i = 0; i < loop; i++)
        {
            if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, 1400) != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }
            ceWifiEsp_sendUartData(ceWifiEsp, (uint8*)(dataBuf + i * 1400), dataSize);
            if (CE_STATUS_SUCCESS != ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000))
            {
                return CE_STATUS_FAILE;
            }
        }
        if (lastData != 0)
        {
            if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, lastData) != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }
            ceWifiEsp_sendUartData(ceWifiEsp,  (uint8*)(dataBuf + loop * 1400), lastData);
            if (CE_STATUS_SUCCESS != ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000))
            {
                return CE_STATUS_FAILE;
            }
        }
    }
    return CE_STATUS_SUCCESS;
}

const CeWifiEspOpBase ceWifiEspOp = { ceWifiEsp_initial, ceWifiEsp_setWorkMode, ceWifiEsp_createWifi, ceWifiEsp_getConnectedDeviceList, ceWifiEsp_getConnectedDeviceCount,
                                        ceWifiEsp_getCanConnectWifiList, ceWifiEsp_getCanConnectWifiCount, ceWifiEsp_checkCanConnectSsidIsExist,
                                        ceWifiEsp_connectWifi, ceWifiEsp_setStaIp, ceWifiEsp_createServer, ceWifiEsp_connectServer,
                                        ceWifiEsp_startUTSendOnClient, ceWifiEsp_stopUTSendOnClient,
                                        ceWifiEsp_getLinkList, ceWifiEsp_getLinkCount, ceWifiEsp_sendData
                                    };

#ifdef __cplusplus
 }
#endif //__cplusplus
