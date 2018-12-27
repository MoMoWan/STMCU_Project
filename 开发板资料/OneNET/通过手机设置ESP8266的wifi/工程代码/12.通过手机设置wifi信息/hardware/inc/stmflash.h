#ifndef _STMFLASH_H_
#define _STMFLASH_H_







#define SSID_ADDRESS		0x0807F800 //�˺ű����ַ	 							��ַ������2�ı���
#define PSWD_ADDRESS		0x0807F8A0 //���뱣���ַ	����ƫ��160�ֽ�(80������)	��ַ������2�ı���



_Bool Flash_NeedErase(void);

void Flash_Read(unsigned int addr, char *rBuf, unsigned short len);

void Flash_Write(unsigned int addr, char *wBuf, unsigned short len);


#endif
