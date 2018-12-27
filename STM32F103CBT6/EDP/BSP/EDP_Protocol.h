#ifndef __EDP_PROTOCOL_H__
#define __EDP_PROTOCOL_H__

#include "stm32f1xx_hal.h"

/*----------------------------��Ϣ����---------------------------------------*/
enum {
	OneNET_CONNREQ=0x10,    //��������
	OneNET_CONNRESP=0x20,   //������Ӧ
	OneNET_PUSHDATA=0x30,   //ת��(͸��)����
	OneNET_UPDATEREQ=0x50,   //��������
	
	OneNET_UPDATERESP=0x60,  //������Ӧ
  OneNET_SAVEDATA=0x80,    //�洢(ת��)����
  OneNET_SAVEACK=0x90,     //�洢ȷ��
  OneNET_CMDREQ=0xA0,      //��������
	
  OneNET_CMDRESP=0xB0,    //������Ӧ
  OneNET_PINGREQ=0xC0,	   //��������
	OneNET_PINGRESP=0xD0,   //������Ӧ
	OneNET_ENCRYPTREQ=0xE0, //��������
	
  OneNET_ENCRYPTRESP=0xF0,	//������Ӧ
	OneNET_UPDATE=0x50       //
};




uint8_t* ONENET_EDPConnect_Msg( char* DevID,char* APIKey,uint16_t CN_Time,uint8_t Msg_Len);





#endif
