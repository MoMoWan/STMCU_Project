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
    ceTMU.recvCallBack(recvBuf,recvCount);
}
/**
  * @brief CeTMUģ���ʼ��
  * @param useType:����TMUģ��ʹ��ʲôͨѶ��ʽ�������ݴ���
  * @param intervalMs:���巢��ʱ����
  * @param recvCallBack:�û����ṩ�Ļص�����
  */
CE_STATUS ceTMU_initial(CE_TUM_USE useType,void (*recvCallBack)(uint8* recvBuf, uint16 recvCount))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.useType = useType;
    if(useType == CE_TMU_USE_WIFI)//�������WIFI����ģʽ
    {
        ceWifiEspOp.initial(&(ceTMU.ceWifiEsp),Uart3);//����Ҫ����ص������������ٵ�
        while(CE_STATUS_SUCCESS != ceWifiEspOp.setWorkMode(&(ceTMU.ceWifiEsp),CE_WIFI_ESP_MODE_STA));
        while(CE_STATUS_SUCCESS != ceWifiEspOp.connectWifi(&(ceTMU.ceWifiEsp),CE_TMU_WIFI_SSID,CE_TMU_WIFI_PWD));
        while (CE_STATUS_SUCCESS != ceWifiEspOp.connectServer(&(ceTMU.ceWifiEsp),0,CE_TMU_WIFI_SERVER_IP,CE_TMU_WIFI_SERVER_PORT,CE_WIFI_ESP_SOCKET_MODE_TCP,ceTMU_recvCallBack));
        ceWifiEspOp.startUTSendOnClient(&(ceTMU.ceWifiEsp));
    }else if(useType == CE_TMU_USE_NRF)//�������2.4G���߹�����ʽ
    {
        ceWlsNrfOp.initial(&(ceTMU.ceWlsNrf),Spi2,PB11GIP,PB10GIP);
    }else if(useType == CE_TMU_USE_BLUE)//�����������������ʽ
    {
        //��ǰ����汾��ʱ��֧����������
        ceBlueHcOp.initial(&(ceTMU.ceBlueHc),Uart1,PB0AGIP,PB1AGIP,PB9GI);
    }
        ceTMU.lastSendTime = ceSystemOp.getSystemTickMs();
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
    //��ǰ����汾��ʱ��֧����������
    }
    ceTMU.lastSendTime = ceSystemOp.getSystemTickMs();

    return     CE_STATUS_SUCCESS;
}
/**
  * @brief ����Ƿ�ͨѶ�ж�
  * @return CE_STATUS_SUCCESS��ͨѶ������ ������ͨѶ�ж�
  */
CE_STATUS ceTMU_checkConnectStatus()
{
    CE_STATUS ceStatus = CE_STATUS_SUCCESS;
    if(ceTMU.sendPackCount >= 20)
    {
        if(ceMathOp.abs((int)(ceTMU.recvPackCount) - (int)(ceTMU.sendPackCount)) >= 20)//��������10֡����û���յ��ظ�������Ϊ�����������
            ceStatus = CE_STATUS_FAILE;    

        ceTMU.sendPackCount=0;
        ceTMU.recvPackCount=0;    
    }
    return ceStatus;
}

/**
  * @brief ��������һ�η������ݵ���ʱ��ʱ����
  * @return ������һ�η������ݵ����ڵ�ʱ��������λms
  */
uint32 ceTMU_getSendIntervalMs(void)   
{
    return  ceSystemOp.getSystemTickMs() - ceTMU.lastSendTime;
}

/**
  * @brief  CeTMUģ�����������
  */
const CeTMUOp ceTMUOp = {ceTMU_initial,ceTMU_sendData,ceTMU_checkConnectStatus,ceTMU_getSendIntervalMs};

#ifdef __cplusplus
 }
#endif //__cplusplus
