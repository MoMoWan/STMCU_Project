/**
  ******************************************************************************
  * @file    CeTMU.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ���ݴ����������WIFI��������2.4Gģ��ĳ�ʼ�������ݷ��ͽ��յȴ���
  ******************************************************************************
  * @attention
  *
  *1)��ֲ��ע�⣺����initial�����У��������ģ��ʹ�õ�����Դ��
  *2)�������ݵ���send����������Byte���鼴�ɣ�
  *3)���յ����ݺ��Զ����ó�ʼ��ʱ�ṩ�Ļص�������δ���κδ����Byte���顣
  *4)���յ����ݺ󣬵��õĻص���������ceTaskOp.mainTask()��ִ�У��뱣֤��main�����е�ceTaskOp.mainTask()�ܹ������ڵ��� 
  * 
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTMU.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus


CeTMU ceTMU;
uint8 nrfAddress[5] = {0x01,0x13,0x5C,0x0C,0x03};//���÷��͵�ַ
/**
  * @brief  ��ͨѶģ��������ݺ���õĻص�����
  * @param  recvBuf:���յ����ݵĻ����ַ
  * @param  recvCount:���յ����ݵĳ���
  */
void ceTMU_recvCallBack(uint8* recvBuf, uint16 recvCount)
{
    ceTMU.recvPackCount++;
    ceFifoOp.write(&(ceTMU.ceFifo),recvBuf,recvCount);
}

/**
  * @brief  ��ͨѶģ��������ݺ���õĻص�����
  * @param  recvBuf:���յ����ݵĻ����ַ
  * @param  recvCount:���յ����ݵĳ���
  */
void ceTMU_taskCallBack(void*  pAddPar)
{
    uint16 recvCount = ceFifoOp.getCanReadSize(&(ceTMU.ceFifo));
    if( recvCount > 0)
        {
        ceTMU.sendCallBack();
        ceFifoOp.read(&(ceTMU.ceFifo),ceTMU.useBuf,recvCount);
                ceTMU.recvCallBack(ceTMU.useBuf,recvCount);            
        }
}

/**
  * @brief  ��ͨѶģ��������ݺ���õĻص�����
  * @param  recvBuf:���յ����ݵĻ����ַ
  * @param  recvCount:���յ����ݵĳ���
  */
void ceTMU_recvCallBackByWifi(uint8 linkID, uint8* recvBuf, uint16 recvCount)
{
    ceTMU.recvPackCount++;
    ceTMU.recvCallBack(recvBuf,recvCount);
}

/**
  * @brief CeTMUģ���ʼ��
  * @param intervalMs:���巢��ʱ����
  * @param recvCallBack:�û����ṩ�Ļص�����
  */
CE_STATUS ceTMU_initialByWifi(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.useType = CE_TMU_USE_WIFI;

    ceWifiEspOp.initial(&(ceTMU.ceWifiEsp),Uart1);
    ceDebugOp.printf("Set wifi work mode...\n");
    while(CE_STATUS_SUCCESS != ceWifiEspOp.setWorkMode(&(ceTMU.ceWifiEsp),CE_WIFI_ESP_MODE_AP))
    {
        ceDebugOp.printf("Set Error, Try again...\n");
        ceSystemOp.delayMs(500);
    };
    ceDebugOp.printf("Create wifi:/nSSID:%s\nPWD:%s\nIP:%s\n...\n",CE_TMU_WIFI_SSID,CE_TMU_WIFI_PWD,CE_TMU_WIFI_SERVER_IP);
    while(CE_STATUS_SUCCESS != ceWifiEspOp.createWifi(&(ceTMU.ceWifiEsp),CE_TMU_WIFI_SERVER_IP, CE_TMU_WIFI_SSID,CE_TMU_WIFI_PWD,1,CE_WIFI_ESP_ECN_WPA2_PSK))
    {
        ceDebugOp.printf("Create wifi Error, Try again...\n");
        ceSystemOp.delayMs(500);            
    };

    ceDebugOp.printf("Create server:/nIP:%s\nMODE:TCP\nPORT:%d\n...\n",CE_TMU_WIFI_SERVER_IP,CE_TMU_WIFI_SERVER_PORT);        
    while(CE_STATUS_SUCCESS != ceWifiEspOp.createServer(&(ceTMU.ceWifiEsp),CE_TMU_WIFI_SERVER_PORT,CE_WIFI_ESP_SOCKET_MODE_TCP,ceTMU_recvCallBackByWifi))
    {
        ceDebugOp.printf("Create server Error, Try again...\n");
        ceSystemOp.delayMs(500);            
    }

    ceDebugOp.printf("Wifi initial finish, wait for client to connect...",CE_TMU_WIFI_SERVER_IP,CE_TMU_WIFI_SERVER_PORT);    

    return CE_STATUS_SUCCESS;
}

/**
  * @brief CeTMUģ���ʼ��
  * @param intervalMs:���巢��ʱ����
  * @param recvCallBack:�û����ṩ�Ļص�����
  */
CE_STATUS ceTMU_initialByNrf(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount), void (*sendCallBack)(void))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.sendCallBack = sendCallBack;
    ceTMU.useType = CE_TMU_USE_NRF;

    ceTMU.ceFifo.buff =ceTMU.fifoBuf;
    ceTMU.ceFifo.buffSize = CE_TMU_NRF_FIFO_SIZE;
    ceFifoOp.initial(&(ceTMU.ceFifo));

    ceTMU.recvTask.callBack = ceTMU_taskCallBack;
    ceTMU.recvTask.ID = 100;

    #ifdef __CE_USE_RTOS__
    ceTMU.recvTask.isNewThread = 0x01;
    ceTMU.recvTask.pAddPar = &ceTMU;
    ceTMU.recvTask.taskName = "TMU Task";
    ceTMU.recvTask.taskPriority = CE_TASK_PRIORITY_H;
    ceTMU.recvTask.taskStackBuf = ceTMU.stackBuf;
    ceTMU.recvTask.taskStackBufSize = CE_TMU_NRF_FIFO_SIZE;
    #endif

    ceTaskOp.registerTask(&(ceTMU.recvTask));
    ceTaskOp.start(&(ceTMU.recvTask));

    ceWlsNrfOp.initial(&(ceTMU.ceWlsNrf),Spi1,PA11GIP,PA12CGI);
        
        ceTMU.sendCallBack ();
    ceWlsNrfOp.recv(&(ceTMU.ceWlsNrf),0,nrfAddress,ceTMU_recvCallBack);//��������
        return CE_STATUS_SUCCESS;
}

/**
  * @brief CeTMUģ���ʼ��
  * @param intervalMs:���巢��ʱ����
  * @param recvCallBack:�û����ṩ�Ļص�����
  */
CE_STATUS ceTMU_initialByBlue(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.useType = CE_TMU_USE_BLUE;
    return CE_STATUS_SUCCESS;
}

/**
  * @brief �������ݣ�ע�⣺�����ڲ����������һ�η������ݵ�ʱ���Ƿ����intervalMs�����С����ֱ�ӷ���
  * @param dataBuf:���ͻ����ַ
  * @param dataCount:���ͻ������ݳ���
  */
CE_STATUS ceTMU_sendData(uint8* dataBuf, uint16 dataCount)
{
    ceTMU.sendPackCount++;
    if(ceTMU.useType == CE_TMU_USE_WIFI)//�������WIFI����ģʽ
    {
        ceWifiEspOp.sendData(&(ceTMU.ceWifiEsp),0,dataBuf,dataCount);
    }else if(ceTMU.useType == CE_TMU_USE_NRF)//�������2.4G���߹�����ʽ
    {
        ceWlsNrfOp.send(&(ceTMU.ceWlsNrf),nrfAddress,dataBuf,dataCount);//�������ݸ�������
        ceWlsNrfOp.recv(&(ceTMU.ceWlsNrf),0,nrfAddress,ceTMU_recvCallBack);//��������
    }else if(ceTMU.useType == CE_TMU_USE_BLUE)//�����������������ʽ
    {
        //��ǰ����汾�ݲ�֧������ͨѶ��ʽ
    }
    return     CE_STATUS_SUCCESS;
} 
/**
  * @brief ����Ƿ�ͨѶ�ж�
  * @return CE_STATUS_SUCCESS��ͨѶ������ ������ͨѶ�ж�
  */
CE_STATUS ceTMU_checkConnectStatus()
{
    CE_STATUS ceStatus = CE_STATUS_SUCCESS;
    if(ceTMU.sendPackCount >= 10)
    {
        if(ceMathOp.abs((int)(ceTMU.recvPackCount) - (int)(ceTMU.sendPackCount)) >= 10)//��������10֡����û���յ��ظ�������Ϊ�����������
            ceStatus = CE_STATUS_FAILE;    
        ceTMU.sendPackCount=0;
        ceTMU.recvPackCount=0;    
    }
    return ceStatus;
}

/**
  * @brief  CeTMUģ�����������
  */
const CeTMUOp ceTMUOp = {ceTMU_initialByWifi,ceTMU_initialByNrf,ceTMU_initialByBlue,ceTMU_sendData,ceTMU_checkConnectStatus};

#ifdef __cplusplus
 }
#endif //__cplusplus
