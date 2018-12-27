/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	onenet.c
	*
	*	���ߣ� 		�ż���
	*
	*	���ڣ� 		2017-05-27
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		OneNETƽ̨Ӧ��ʾ��
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//�����豸
#include "net_device.h"

//Э���ļ�
#include "onenet.h"
#include "fault.h"
#include "httpkit.h"

//Ӳ������
#include "usart.h"

//ͼƬ�����ļ�
#include "image_2k.h"

//C��
#include <string.h>
#include <stdio.h>


ONETNET_INFO onenet_info = {"5616708", "kPl=kf99QxL2acVvVCHssUPWZKs=",
							"183.230.40.33", "80",
							10,
							NULL, 0, 0, 0, 0};



//==========================================================
//	�������ƣ�	OneNet_SendData
//
//	�������ܣ�	�ϴ����ݵ�ƽ̨
//
//	��ڲ�����	type���������ݵĸ�ʽ
//				devid���豸ID
//				apikey���豸apikey
//				streamArray��������
//				streamArrayNum������������
//
//	���ز�����	0-�ɹ�		1-ʧ��
//
//	˵����		
//==========================================================
unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt)
{
	
	HTTP_PACKET_STRUCTURE httpPacket = {NULL, 0, 0, 0};										//Э���
	
	_Bool status = SEND_TYPE_OK;
	short body_len = 0;
	
	if(!onenet_info.netWork)
		return SEND_TYPE_DATA;
	
	onenet_info.errCount++;
	
	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-HTTP_TYPE%d\r\n", type);
	
	body_len = DSTREAM_GetDataStream_Body_Measure(type, streamArray, streamArrayCnt, 0);	//��ȡ��ǰ��Ҫ���͵����������ܳ���
	if(body_len > 0)
	{
		if(HTTP_Post_PacketSaveData(devid, apikey, body_len, NULL, (SaveDataType)type, &httpPacket) == 0)
		{
			body_len = DSTREAM_GetDataStream_Body(type, streamArray, streamArrayCnt, httpPacket._data, httpPacket._size, httpPacket._len);
			
			if(body_len > 0)
			{
				httpPacket._len += body_len;
				UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", httpPacket._len);
				//NET_DEVICE_SendData(httpPacket._data, httpPacket._len);					//�ϴ����ݵ�ƽ̨
				NET_DEVICE_AddDataSendList(httpPacket._data, httpPacket._len);				//��������
			}
			else
				UsartPrintf(USART_DEBUG, "WARN:	DSTREAM_GetDataStream_Body Failed\r\n");
				
			HTTP_DeleteBuffer(&httpPacket);													//ɾ��
		}
		else
			UsartPrintf(USART_DEBUG, "WARN:	HTTP_NewBuffer Failed\r\n");
	}
	else if(body_len < 0)
		return SEND_TYPE_OK;
	else
		status = SEND_TYPE_DATA;
	
	net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;										//����֮��������
	
	return status;
	
}

//==========================================================
//	�������ƣ�	OneNet_RegisterDevice
//
//	�������ܣ�	ע���豸
//
//	��ڲ�����	apikey��master-key(��ƷAPIKEY)
//				title���豸��
//				auth_info:�豸�ļ�Ȩ��Ϣ
//				desc��������Ϣ
//				Private���Ƿ񹫿�
//
//	���ز�����	0-�ɹ�		1-ʧ��
//
//	˵����		��HTTPЭ����豸�����ô˺���ǰ��Ҫ�Ͽ���ǰ���ӣ�������HTTPЭ���ip
//==========================================================
unsigned char OneNet_RegisterDevice(const char *apikey, const char *title, const char *auth_info, const char *desc, const char *Private)
{
	
	HTTP_PACKET_STRUCTURE httpPacket = {NULL, 0, 0, 0};							//Э���

	if(!onenet_info.netWork)														//�������Ϊ���� �� ��Ϊ�����շ�ģʽ
		return SEND_TYPE_REGISTER;
	
	if(HTTP_Post_PacketDeviceRegister(apikey, title, auth_info, desc, Private, &httpPacket) == 0)
	{
		//NET_DEVICE_SendData(httpPacket._data, httpPacket._len);				//�ϴ����ݵ�ƽ̨
		NET_DEVICE_AddDataSendList(httpPacket._data, httpPacket._len);				//��������
		HTTP_DeleteBuffer(&httpPacket);											//ɾ��
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	�������ƣ�	OneNet_Status
//
//	�������ܣ�	����״̬���
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void OneNet_Status(void)
{
	
	unsigned char errType = 0;
	
	if(!onenet_info.netWork)														//�������Ϊ����
		return;
	
	errType = NET_DEVICE_Check();												//�����豸״̬���
	if(errType == CHECK_CLOSED || errType == CHECK_GOT_IP)
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
	else if(errType == CHECK_NO_DEVICE || errType == CHECK_NO_CARD)
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_3;
	else
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;
	
}

//==========================================================
//	�������ƣ�	OneNET_CmdHandle
//
//	�������ܣ�	��ȡƽ̨rb�е�����
//
//	��ڲ�����	��
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void OneNET_CmdHandle(void)
{
	
	unsigned char *dataPtr = NULL, *ipdPtr = NULL;		//����ָ��

	dataPtr = NET_DEVICE_Read();						//�ȴ�����

	if(dataPtr != NULL)									//������Ч
	{
		ipdPtr = NET_DEVICE_GetIPD(dataPtr);			//����Ƿ���ƽ̨����
		if(ipdPtr != NULL)
		{
			net_device_info.send_ok = 1;
			
			if(net_device_info.netWork)
				onenet_info.cmd_ptr = ipdPtr;			//���ƽ̨�·�����
			else
				net_device_info.cmd_ipd = (char *)ipdPtr;
		}
		else
		{
			if(strstr((char *)dataPtr, "SEND OK") != NULL)
			{
				net_device_info.send_ok = 1;
			}
			else if(strstr((char *)dataPtr, "CLOSE") != NULL && net_device_info.netWork)
			{
				UsartPrintf(USART_DEBUG, "WARN:	���ӶϿ���׼������\r\n");
				
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
			}
			else
				NET_DEVICE_CmdHandle((char *)dataPtr);
		} 
	}

}

//==========================================================
//	�������ƣ�	OneNet_RevPro
//
//	�������ܣ�	ƽ̨�������ݼ��
//
//	��ڲ�����	dataPtr��ƽ̨���ص�����
//
//	���ز�����	��
//
//	˵����		
//==========================================================
void OneNet_RevPro(unsigned char *dataPtr)
{
	
	char *devid = NULL;

	if(strstr((char *)dataPtr, "CLOSED"))
	{
		UsartPrintf(USART_DEBUG, "TCP CLOSED\r\n");
		
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
		
		onenet_info.errCount++;
	}
	else
	{
		//������������Ƿ��ͳɹ�
		if(strstr((char *)dataPtr, "succ"))
		{
			UsartPrintf(USART_DEBUG, "Tips:	Send OK\r\n");
			
			if(strstr((char *)dataPtr, "device_id"))
			{
				if(HTTP_UnPacketDeviceRegister(dataPtr, &devid) == 0)
				{
					UsartPrintf(USART_DEBUG, "devid: %s\r\n", devid);
					HTTP_FreeBuffer(devid);
				}
			}
			
			onenet_info.errCount = 0;
		}
		else
		{
			UsartPrintf(USART_DEBUG, "Tips:	Send Err\r\n");
			onenet_info.errCount++;
		}
	}

}
