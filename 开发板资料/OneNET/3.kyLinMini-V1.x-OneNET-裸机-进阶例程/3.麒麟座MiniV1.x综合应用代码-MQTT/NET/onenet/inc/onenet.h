#ifndef _ONENET_H_
#define _ONENET_H_


#include "dstream.h"




typedef struct
{

    char devID[15];
    char apiKey[35];
	
	char proID[10];
	char auif[50];
	
	char ip[16];
	char port[8];
	
	const unsigned char protocol;	//Э�����ͺ�		1-edp	2-nwx	3-jtext		4-Hiscmd
									//				5-jt808			6-modbus	7-mqtt
									//				8-gr20			9-reg		10-HTTP(�Զ���)
	
	unsigned char *cmd_ptr;			//ƽ̨�·�������
	
	unsigned char netWork : 1;		//1-OneNET����ɹ�		0-OneNET����ʧ��
	unsigned char sendData : 3;
	unsigned char errCount : 3;		//�������
	unsigned char heartBeat : 1;	//����

} ONETNET_INFO;

extern ONETNET_INFO onenet_info;





#define CHECK_CONNECTED			0	//������
#define CHECK_CLOSED			1	//�ѶϿ�
#define CHECK_GOT_IP			2	//�ѻ�ȡ��IP
#define CHECK_NO_DEVICE			3	//���豸
#define CHECK_INITIAL			4	//��ʼ��״̬
#define CHECK_NO_CARD			5	//û��sim��
#define CHECK_NO_ERR			255 //

#define SEND_TYPE_OK			0	//
#define SEND_TYPE_DATA			1	//
#define SEND_TYPE_HEART			2	//
#define SEND_TYPE_PUBLISH		3	//
#define SEND_TYPE_SUBSCRIBE		4	//
#define SEND_TYPE_UNSUBSCRIBE	5	//




void OneNet_DevLink(const char* devid, const char *proid, const char* auth_info);

_Bool OneNet_DisConnect(void);

unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt);

unsigned char OneNet_Subscribe(const char *topics[], unsigned char topic_cnt);

unsigned char OneNet_UnSubscribe(const char *topics[], unsigned char topic_cnt);

unsigned char OneNet_Publish(const char *topic, const char *msg);

unsigned char OneNet_SendData_Heart(void);

_Bool OneNet_Check_Heart(void);

void OneNET_CmdHandle(void);

void OneNet_RevPro(unsigned char *dataPtr);

#endif
