/**
  ******************************************************************************
  * @file    CePackage.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   ������CePackageģ�������ͷ�ļ�,�������ݵĴ���������
  ******************************************************************************
  * @attention
  *
  *1)��Ӷ����������ֱ��CePackageSend��CePackageRecv����Ӽ��ɣ�ϵͳ�Զ�����ṹ�峤��
  *2)��ӵı���һ��Ϊint32�����������޷�Ԥ��Ĺ���
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CePackage.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

uint8 cePackageSendBuf[CE_PACKAGE_SEND_BUF_SIZE];//�������ݻ���

/**
  * @brief  cePackageSendģ���ʼ�����Խṹ���е����ݽ�����0����
  * @param cePackageSend:CePackageSend���Զ���ָ��
  */
CE_STATUS   cePackage_initialSend(CePackageSend* cePackageSend)
{
    int i;
    int32* temp = (int32*)(cePackageSend);
    for(i=0;i<sizeof(CePackageRecv)/sizeof(int32);i++)
    {
        *temp = 0;
        temp++;
    }

    for(i=0;i<CE_PACKAGE_SEND_BUF_SIZE;i++)
        cePackageSendBuf[i] = 0x00;

    return CE_STATUS_SUCCESS;
}
/**
  * @brief cePackageRecvģ���ʼ�����Խṹ���е����ݽ�����0����
  * @param cePackageRecv:CePackageRecv���Զ���ָ��
  */
CE_STATUS   cePackage_initialRecv(CePackageRecv* cePackageRecv)
{
    int i;
    int32* temp = (int32*)(cePackageRecv);
    for(i=0;i<sizeof(CePackageRecv)/sizeof(int32);i++)
    {
        *temp = 0;
        temp++;
    }
    return CE_STATUS_SUCCESS;
}
/**
  * @brief ��cePackageSend�ṹ����д�������ش�����ֱ�ӷ���byte����
  * @param cePackageSend:CePackageSend���Զ���ָ��
  * @param ctlStatus:���˻���ǰ�ܿ�״̬����ӦcePackageRecv���status
  * @return ������ֱ�ӷ��͵�byte���飬����ΪCE_PACKAGE_SEND_BUF_SIZE
  */
uint8*  cePackage_dealSend(CePackageSend* cePackageSend, uint32 ctlStatus)
{
    int i,j;
    int32* temp = (int32*)(cePackageSend);
    uint8 sum=0;
    uint16 packCount = 0;
    if((ctlStatus & CE_CTL_TYPE_STATION)  != 0)//����ǵ���վ����Ʒ�ʽ������ȫ�����ݣ��������Ʒ�ʽ��ֻ����һ�����ݼ��ɡ�
        packCount = (CE_PACKAGE_SEND_BUF_SIZE / CE_PACKAGE_PACK_SIZE);
    else 
        packCount = 1;
    for(j=0;j< packCount;j++)
    {
        cePackageSendBuf[0+j*CE_PACKAGE_PACK_SIZE] = 0x55;  //֡ͷ0x55
        cePackageSendBuf[1+j*CE_PACKAGE_PACK_SIZE] = j;     //��N��
        sum = 0x55+j;
        for(i=2;i< CE_PACKAGE_PACK_SIZE-2;i+=4)
        {
            if(temp >= (int32*)(cePackageSend) + sizeof(CePackageSend)/sizeof(int32))
                break;
            cePackageSendBuf[i+0+j*CE_PACKAGE_PACK_SIZE] = (*temp)&0xFF;
            cePackageSendBuf[i+1+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 8)&0xFF;
            cePackageSendBuf[i+2+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 16)&0xFF;
            cePackageSendBuf[i+3+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 24)&0xFF;
            temp++;
            sum = (sum + cePackageSendBuf[i+j*CE_PACKAGE_PACK_SIZE] + cePackageSendBuf[i+1+j*CE_PACKAGE_PACK_SIZE]) + cePackageSendBuf[i+2+j*CE_PACKAGE_PACK_SIZE]+cePackageSendBuf[i+3+j*CE_PACKAGE_PACK_SIZE] ;//������ۼ�
        }
        cePackageSendBuf[CE_PACKAGE_PACK_SIZE+j*CE_PACKAGE_PACK_SIZE-2] = sum;//ǰ�����ݵĺϣ�����У�����ݵ���ȷ��
        cePackageSendBuf[CE_PACKAGE_PACK_SIZE+j*CE_PACKAGE_PACK_SIZE-1] = 0xFE;//֡β

    }
    return cePackageSendBuf;
}
/**
  * @brief ��recvBuf�е����ݽ��в����������������Ľ�����µ��ṹ��cePackageRecv
  * @param cePackageRecv:CePackageRecv���Զ���ָ��
  * @param recvBuf:�������ݻ����ַ
  * @param recvCount:�������ݻ��泤��
  * @return ����CE_STATUS_SUCCESS������ɹ�������ʧ��
  */
CE_STATUS    cePackage_dealRecv(CePackageRecv* cePackageRecv, uint8* recvBuf, uint16 recvCount)
{
    int i=0,j;
    int packIndex = 0;
    int32* temp = (int32*)(cePackageRecv);
    uint8 sum=0;
    if( recvCount % CE_PACKAGE_PACK_SIZE != 0 )
        return CE_STATUS_FAILE;
    for(j=0;j< recvCount/CE_PACKAGE_PACK_SIZE;j++)
    {
        if(recvBuf[0+j*CE_PACKAGE_PACK_SIZE] != 0x55 || recvBuf[CE_PACKAGE_PACK_SIZE-1+j*CE_PACKAGE_PACK_SIZE] != 0xFE)//���֡ͷ��֡β
            return CE_STATUS_FAILE;//֡ͷ�����ϣ�ֱ�ӷ��أ������д���

        sum = 0;
        for(i=j*CE_PACKAGE_PACK_SIZE;i<j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2;i++)        //���м���ϼ��
            sum += recvBuf[i];

        if(sum != recvBuf[j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2])
            return CE_STATUS_FAILE;//У��ϲ����ϣ�ֱ�ӷ��أ������д���

        packIndex = recvBuf[j*CE_PACKAGE_PACK_SIZE+1];

        if(packIndex >= CE_PACKAGE_RECV_BUF_SIZE/CE_PACKAGE_PACK_SIZE)
            break;

        temp = (int32*)(cePackageRecv) + packIndex * (CE_PACKAGE_PACK_SIZE-4)/4;

        for(i=j*CE_PACKAGE_PACK_SIZE+2;i<j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2;i+=4)
        {
            if(temp >= (int32*)(cePackageRecv) + sizeof(CePackageRecv)/sizeof(int32))
                break;

            *temp = (int32)(((0x000000FF & recvBuf[i+3]) << 24) |((0x000000FF & recvBuf[i+2]) << 16) |((0x000000FF & recvBuf[i+1]) << 8) | (0x000000FF & recvBuf[i]));
            temp++;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  CePackageģ�����������
  */
const CePackageOp cePackageOp = {cePackage_initialSend,cePackage_initialRecv,cePackage_dealSend,cePackage_dealRecv};

#ifdef __cplusplus
}
#endif //__cplusplus
