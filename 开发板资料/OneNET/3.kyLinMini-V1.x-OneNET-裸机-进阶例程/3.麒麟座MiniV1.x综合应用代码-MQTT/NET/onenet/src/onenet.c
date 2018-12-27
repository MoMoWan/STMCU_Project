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
#include "mqttkit.h"

//Ӳ������
#include "usart.h"
#include "delay.h"
#include "led.h"

//ͼƬ�����ļ�
#include "image_2k.h"

//C��
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


ONETNET_INFO onenet_info = {"5616839", "6Uvvwf=ab7O3ErvEKxyFILZxZ0s=",
							"77247", "test",
							"183.230.40.39", "6002",
							7,
							NULL, 0, 0, 0, 1};



//==========================================================
//	�������ƣ�	OneNet_DevLink
//
//	�������ܣ�	��onenet��������
//
//	��ڲ�����	devid�������豸��devid
//				proid����ƷID
//				auth_key�������豸��masterKey��apiKey���豸��Ȩ��Ϣ
//
//	���ز�����	��
//
//	˵����		��onenetƽ̨�������ӣ��ɹ������oneNetInfo.netWork����״̬��־
//==========================================================
void OneNet_DevLink(const char* devid, const char *proid, const char* auth_info)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					//Э���
	
	unsigned char timeOut = 200;
	
	UsartPrintf(USART_DEBUG, "OneNet_DevLink\r\n"
							"PROID: %s,	AUIF: %s,	DEVID:%s\r\n"
                        , proid, auth_info, devid);
	
	if(MQTT_PacketConnect(proid, auth_info, devid, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
		NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);			//�ϴ�ƽ̨
		//NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//��������
		
		MQTT_DeleteBuffer(&mqttPacket);									//ɾ��
		
		while(!onenet_info.netWork && --timeOut)
			DelayXms(10);
	}
	else
		UsartPrintf(USART_DEBUG, "WARN:	MQTT_PacketConnect Failed\r\n");
	
	if(onenet_info.netWork)											//�������ɹ�
	{
		onenet_info.errCount = 0;
	}
	else
	{
		if(++onenet_info.errCount >= 5)								//��������趨�����󣬻�δ����ƽ̨
		{
			onenet_info.netWork = 0;
			onenet_info.errCount = 0;
			
			net_fault_info.net_fault_level =
			net_fault_info.net_fault_level_r =
			NET_FAULT_LEVEL_3;										//����ȼ�3
		}
	}
	
}

//==========================================================
//	�������ƣ�	OneNet_DisConnect
//
//	�������ܣ�	��ƽ̨�Ͽ�����
//
//	��ڲ�����	��
//
//	���ز�����	0-�ɹ�		1-ʧ��
//
//	˵����		
//==========================================================
_Bool OneNet_DisConnect(void)
{

	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//Э���

	if(!onenet_info.netWork)
		return 1;
	
	if(MQTT_PacketDisConnect(&mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//��ƽ̨���Ͷ�������
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//��������
		
		MQTT_DeleteBuffer(&mqttPacket);											//ɾ��
	}
	
	return 0;

}

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
//	���ز�����	SEND_TYPE_OK-���ͳɹ�	SEND_TYPE_DATA-��Ҫ����
//
//	˵����		
//==========================================================
unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};											//Э���
	
	_Bool status = SEND_TYPE_OK;
	short body_len = 0;
	
	if(!onenet_info.netWork)
		return SEND_TYPE_DATA;
	
	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-MQTT_TYPE%d\r\n", type);
	
	body_len = DSTREAM_GetDataStream_Body_Measure(type, streamArray, streamArrayCnt, 0);		//��ȡ��ǰ��Ҫ���͵����������ܳ���
	if(body_len > 0)
	{
		if(MQTT_PacketSaveData(devid, body_len, NULL, (uint8)type, &mqttPacket) == 0)
		{
			body_len = DSTREAM_GetDataStream_Body(type, streamArray, streamArrayCnt, mqttPacket._data, mqttPacket._size, mqttPacket._len);
			
			if(body_len > 0)
			{
				mqttPacket._len += body_len;
				UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", mqttPacket._len);
				//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);						//�ϴ����ݵ�ƽ̨
				NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);					//��������
			}
			else
				UsartPrintf(USART_DEBUG, "WARN:	DSTREAM_GetDataStream_Body Failed\r\n");
				
			MQTT_DeleteBuffer(&mqttPacket);														//ɾ��
		}
		else
			UsartPrintf(USART_DEBUG, "WARN:	MQTT_NewBuffer Failed\r\n");
	}
	else if(body_len < 0)
		return SEND_TYPE_OK;
	else
		status = SEND_TYPE_DATA;
	
	net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;										//����֮��������
	
	return status;
	
}

//==========================================================
//	�������ƣ�	OneNet_HeartBeat
//
//	�������ܣ�	�������
//
//	��ڲ�����	��
//
//	���ز�����	SEND_TYPE_OK-���ͳɹ�	SEND_TYPE_DATA-��Ҫ����
//
//	˵����		
//==========================================================
unsigned char OneNet_SendData_Heart(void)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//Э���
	
	if(!onenet_info.netWork)										//�������Ϊ����
		return SEND_TYPE_HEART;
	
	if(MQTT_PacketPing(&mqttPacket))
		return SEND_TYPE_HEART;
	
	onenet_info.heartBeat = 0;
	
	//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);		//��ƽ̨�ϴ���������
	NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//��������
	
	MQTT_DeleteBuffer(&mqttPacket);								//ɾ��
	
	return SEND_TYPE_OK;

}

//==========================================================
//	�������ƣ�	OneNet_HeartBeat_Check
//
//	�������ܣ�	������������������
//
//	��ڲ�����	��
//
//	���ز�����	0-�ɹ�	1-�ȴ�
//
//	˵����		���ڵ���ʱ����runCountÿ���˺�������һ�ε�ʱ������
//				�ﵽ�趨���޼��������־λ�Ƿ����
//				����ʱ����Բ���̫��ȷ
//==========================================================
_Bool OneNet_Check_Heart(void)
{
	
	static unsigned char runCount = 0;
	
	if(!onenet_info.netWork)
		return 1;

	if(onenet_info.heartBeat == 1)
	{
		runCount = 0;
		onenet_info.errCount = 0;
		
		return 0;
	}
	
	if(++runCount >= 40)
	{
		runCount = 0;
		
		UsartPrintf(USART_DEBUG, "HeartBeat TimeOut: %d\r\n", onenet_info.errCount);
		onenet_info.sendData = SEND_TYPE_HEART;		//������������
		
		if(++onenet_info.errCount >= 3)
		{
			unsigned char errType = 0;
			
			onenet_info.errCount = 0;
			
			errType = NET_DEVICE_Check();												//�����豸״̬���
			if(errType == CHECK_CONNECTED || errType == CHECK_CLOSED || errType == CHECK_GOT_IP)
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
			else if(errType == CHECK_NO_DEVICE || errType == CHECK_NO_CARD)
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_3;
			else
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;
		}
	}
	
	return 1;

}

//==========================================================
//	�������ƣ�	OneNet_Publish
//
//	�������ܣ�	������Ϣ
//
//	��ڲ�����	topic������������
//				msg����Ϣ����
//
//	���ز�����	SEND_TYPE_OK-�ɹ�	SEND_TYPE_PUBLISH-��Ҫ����
//
//	˵����		
//==========================================================
unsigned char OneNet_Publish(const char *topic, const char *msg)
{

	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//Э���

	if(!onenet_info.netWork)
		return SEND_TYPE_PUBLISH;
	
	UsartPrintf(USART_DEBUG, "Publish Topic: %s, Msg: %s\r\n", topic, msg);
	
	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), MQTT_QOS_LEVEL2, 0, 1, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//��ƽ̨���Ͷ�������
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//��������
		
		MQTT_DeleteBuffer(&mqttPacket);											//ɾ��
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	�������ƣ�	OneNet_Subscribe
//
//	�������ܣ�	����
//
//	��ڲ�����	topics�����ĵ�topic
//				topic_cnt��topic����
//
//	���ز�����	SEND_TYPE_OK-�ɹ�	SEND_TYPE_SUBSCRIBE-��Ҫ�ط�
//
//	˵����		
//==========================================================
unsigned char OneNet_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//Э���

	if(!onenet_info.netWork)
		return SEND_TYPE_SUBSCRIBE;
	
	for(; i < topic_cnt; i++)
		UsartPrintf(USART_DEBUG, "Subscribe Topic: %s\r\n", topics[i]);
	
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL2, topics, topic_cnt, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//��ƽ̨���Ͷ�������
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//��������
		
		MQTT_DeleteBuffer(&mqttPacket);											//ɾ��
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	�������ƣ�	OneNet_UnSubscribe
//
//	�������ܣ�	ȡ������
//
//	��ڲ�����	topics�����ĵ�topic
//				topic_cnt��topic����
//
//	���ز�����	SEND_TYPE_OK-���ͳɹ�	SEND_TYPE_UNSUBSCRIBE-��Ҫ�ط�
//
//	˵����		
//==========================================================
unsigned char OneNet_UnSubscribe(const char *topics[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//Э���

	if(!onenet_info.netWork)
		return SEND_TYPE_UNSUBSCRIBE;
	
	for(; i < topic_cnt; i++)
		UsartPrintf(USART_DEBUG, "UnSubscribe Topic: %s\r\n", topics[i]);
	
	if(MQTT_PacketUnSubscribe(MQTT_UNSUBSCRIBE_ID, topics, topic_cnt, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//��ƽ̨����ȡ����������
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//��������
		
		MQTT_DeleteBuffer(&mqttPacket);											//ɾ��
	}
	
	return SEND_TYPE_OK;

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
				OneNet_RevPro(ipdPtr);					//���д���
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
void OneNet_RevPro(unsigned char *cmd)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};								//Э���
	
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	unsigned char type = 0;
	unsigned char qos = 0;
	static unsigned short pkt_id = 0;
	
	short result = 0;

	char *dataPtr = NULL;
	char numBuf[10];
	int num = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		case MQTT_PKT_CONNACK:
		
			switch(MQTT_UnPacketConnectAck(cmd))
			{
				case 0:
					UsartPrintf(USART_DEBUG, "Tips:	���ӳɹ�\r\n");
					onenet_info.netWork = 1;
				break;
				
				case 1:UsartPrintf(USART_DEBUG, "WARN:	����ʧ�ܣ�Э�����\r\n");break;
				case 2:UsartPrintf(USART_DEBUG, "WARN:	����ʧ�ܣ��Ƿ���clientid\r\n");break;
				case 3:UsartPrintf(USART_DEBUG, "WARN:	����ʧ�ܣ�������ʧ��\r\n");break;
				case 4:UsartPrintf(USART_DEBUG, "WARN:	����ʧ�ܣ��û������������\r\n");break;
				case 5:UsartPrintf(USART_DEBUG, "WARN:	����ʧ�ܣ��Ƿ�����(����token�Ƿ�)\r\n");break;
				
				default:UsartPrintf(USART_DEBUG, "ERR:	����ʧ�ܣ�δ֪����\r\n");break;
			}
		
		break;
		
		case MQTT_PKT_PINGRESP:
		
			UsartPrintf(USART_DEBUG, "Tips:	HeartBeat OK\r\n");
			onenet_info.heartBeat = 1;
		
		break;
		
		case MQTT_PKT_CMD:															//�����·�
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload);				//���topic����Ϣ��
			if(result == 0)
			{
				UsartPrintf(USART_DEBUG, "cmdid: %s, req: %s\r\n", cmdid_topic, req_payload);
				
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//����ظ����
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send CmdResp\r\n");
					
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);		//�ظ�����
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//��������
					MQTT_DeleteBuffer(&mqttPacket);									//ɾ��
				}
			}
		
		break;
			
		case MQTT_PKT_PUBLISH:														//���յ�Publish��Ϣ
		
			result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &req_payload, &qos, &pkt_id);
			if(result == 0)
			{
				UsartPrintf(USART_DEBUG, "topic: %s\r\npayload: %s\r\n", cmdid_topic, req_payload);
				
				switch(qos)
				{
					case 1:															//�յ�publish��qosΪ1���豸��Ҫ�ظ�Ack
					
						if(MQTT_PacketPublishAck(pkt_id, &mqttPacket) == 0)
						{
							UsartPrintf(USART_DEBUG, "Tips:	Send PublishAck\r\n");
							//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
							NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//��������
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					case 2:															//�յ�publish��qosΪ2���豸�Ȼظ�Rec
																					//ƽ̨�ظ�Rel���豸�ٻظ�Comp
						if(MQTT_PacketPublishRec(pkt_id, &mqttPacket) == 0)
						{
							UsartPrintf(USART_DEBUG, "Tips:	Send PublishRec\r\n");
							//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
							NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//��������
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					default:
						break;
				}
			}
		
		break;
			
		case MQTT_PKT_PUBACK:														//����Publish��Ϣ��ƽ̨�ظ���Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Publish Send OK\r\n");
			
		break;
			
		case MQTT_PKT_PUBREC:														//����Publish��Ϣ��ƽ̨�ظ���Rec���豸��ظ�Rel��Ϣ
		
			if(MQTT_UnPacketPublishRec(cmd) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishRec\r\n");
				if(MQTT_PacketPublishRel(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send PublishRel\r\n");
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//��������
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
			
		case MQTT_PKT_PUBREL:														//�յ�Publish��Ϣ���豸�ظ�Rec��ƽ̨�ظ���Rel���豸���ٻظ�Comp
			
			if(MQTT_UnPacketPublishRel(cmd, pkt_id) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishRel\r\n");
				if(MQTT_PacketPublishComp(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send PublishComp\r\n");
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//��������
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
		
		case MQTT_PKT_PUBCOMP:														//����Publish��Ϣ��ƽ̨����Rec���豸�ظ�Rel��ƽ̨�ٷ��ص�Comp
		
			if(MQTT_UnPacketPublishComp(cmd) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishComp\r\n");
			}
		
		break;
			
		case MQTT_PKT_SUBACK:														//����Subscribe��Ϣ��Ack
		
			if(MQTT_UnPacketSubscribe(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Subscribe OK\r\n");
			else
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Subscribe Err\r\n");
		
		break;
			
		case MQTT_PKT_UNSUBACK:														//����UnSubscribe��Ϣ��Ack
		
			if(MQTT_UnPacketUnSubscribe(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT UnSubscribe OK\r\n");
			else
				UsartPrintf(USART_DEBUG, "Tips:	MQTT UnSubscribe Err\r\n");
		
		break;
		
		default:
			result = -1;
		break;
	}
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req_payload, '}');					//����'}'

	if(dataPtr != NULL)									//����ҵ���
	{
		dataPtr++;
		
		while(*dataPtr >= '0' && *dataPtr <= '9')		//�ж��Ƿ����·��������������
		{
			numBuf[num++] = *dataPtr++;
		}
		
		num = atoi((const char *)numBuf);				//תΪ��ֵ��ʽ
		
		if(strstr((char *)req_payload, "redled"))		//����"redled"
		{
			if(num == 1)								//�����������Ϊ1������
			{
				Led5_Set(LED_ON);
			}
			else if(num == 0)							//�����������Ϊ0�������
			{
				Led5_Set(LED_OFF);
			}
		}
														//��ͬ
		else if(strstr((char *)req_payload, "greenled"))
		{
			if(num == 1)
			{
				Led4_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led4_Set(LED_OFF);
			}
		}
		else if(strstr((char *)req_payload, "yellowled"))
		{
			if(num == 1)
			{
				Led3_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led3_Set(LED_OFF);
			}
		}
		else if(strstr((char *)req_payload, "blueled"))
		{
			if(num == 1)
			{
				Led2_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led2_Set(LED_OFF);
			}
		}
	}

	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
		onenet_info.sendData = SEND_TYPE_DATA;
	}

}
