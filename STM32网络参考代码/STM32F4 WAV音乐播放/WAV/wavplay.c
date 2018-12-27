#include "wavplay.h"
#include "delay.h"
#include "timer.h"
#include "lcd.h"
#include "dac.h"
#include "key.h"

WAV_file wav1;//wav�ļ�
u8 wav_buf[1024];
u16 DApc;
u8 CHanalnum;
u8 Bitnum;
u8 DACdone;
extern u8 volume;
FileInfoStruct *CurFile;//��ǰ����/�������ļ�	 
u8 WAV_Init(u8* pbuf)//��ʼ������ʾ�ļ���Ϣ
{
	if(Check_Ifo(pbuf,"RIFF"))return 1;//RIFF��־����
	wav1.wavlen=Get_num(pbuf+4,4);//�ļ����ȣ�����ƫ��4byte
	if(Check_Ifo(pbuf+8,"WAVE"))return 2;//WAVE��־����
	if(Check_Ifo(pbuf+12,"fmt "))return 3;//fmt��־����
	wav1.formart=Get_num(pbuf+20,2);//��ʽ���
	wav1.CHnum=Get_num(pbuf+22,2);//ͨ����
	CHanalnum=wav1.CHnum;
	wav1.SampleRate=Get_num(pbuf+24,4);//������
	wav1.speed=Get_num(pbuf+28,4);//��Ƶ��������
	wav1.ajust=Get_num(pbuf+32,2);//���ݿ������
	wav1.SampleBits=Get_num(pbuf+34,2);//��������λ��
	Bitnum=wav1.SampleBits;
	if(Check_Ifo(pbuf+36,"data"))return 4;//data��־����
	wav1.DATAlen=Get_num(pbuf+40,4);//���ݳ���	
	///////////////////////////////////////////////
	if(wav1.wavlen<0x100000)
	{
		LCD_ShowNum(170,30,(wav1.wavlen)>>10,3,16);//�ļ�����
		LCD_ShowString(200,30,"Kb");
	}
	else
	{
		LCD_ShowNum(170,30,(wav1.wavlen)>>20,3,16);//�ļ�����
		LCD_ShowString(200,30,"Mb");
	}
	if(wav1.formart==1)LCD_ShowString(170,50,"WAV PCM");
	if(wav1.CHnum==1)LCD_ShowString(170,70,"single");
	else LCD_ShowString(170,70,"stereo");
	LCD_ShowNum(170,90,(wav1.SampleRate)/1000,3,16);//������
	LCD_ShowString(200,90,"KHz");
	LCD_ShowNum(170,110,(wav1.speed)/1000,3,16);//�����ٶ�
	LCD_ShowString(200,110,"bps");
	LCD_ShowNum(177,130,wav1.SampleBits,2,16);//��������λ��
	LCD_ShowString(200,130,"bit");
	return 0;
}

u8 Playwav(FileInfoStruct *FileName)
{
	u16 i,times;
	CurFile=FileName;
	if(CurFile->F_Type!=T_WAV)return 1;
	F_Open(CurFile);
	F_Read(CurFile,wav_buf);//�ȶ�512�ֽڵ�
	F_Read(CurFile,wav_buf+512);//�ٶ�512�ֽ�
	while(WAV_Init(wav_buf))LCD_ShowString(35,70,"format illegal!");
	//���ݲ����ʣ�wav1.SampleRate�����ö�ʱ�������ж��н���DAת��
	DACdone=0;
	DApc=44;//DAת����ַ(����ͷ��Ϣ)
	LCD_DrawRectangle(18,258,222,272);//���ȿ�
	LCD_Fill(20,260,220,270,WHITE);//������
	Timerx_Init(1000000/wav1.SampleRate,72);//1MHz�ļ���Ƶ��,�����Ͳ�����һ�����ж�Ƶ��
	times=(wav1.DATAlen>>10)-1;
	for(i=0;i<times;i++)//ѭ��һ��ת��1KB����
	{	
		while(!DACdone);//�ȴ�ǰ��512�ֽ�ת�����
		DACdone=0;
		F_Read(CurFile,wav_buf);//��512�ֽ�
		LCD_Fill(20,260,20+(200*i/times),270,BLUE);//������
		while(!DACdone);//�ȴ�ǰ��512�ֽ�ת�����
		DACdone=0;
		F_Read(CurFile,wav_buf+512);//��512�ֽ�	
		if((KEY1&KEY2&KEY3&KEY4)==0)
		{			
			if((KEY1&KEY2)==0){TIM3->CR1&=~0x01;break;}//�ض�ʱ��
			else if(KEY3==0&&volume<255)volume++;
			else if(KEY4==0&&volume>10)volume--;				  
		}
	}
	return 0;
}

u8 Check_Ifo(u8* pbuf1,u8* pbuf2)
{
	u8 i;
	for(i=0;i<4;i++)if(pbuf1[i]!=pbuf2[i])return 1;//��ͬ
	return 0;//��ͬ
}

u32 Get_num(u8* pbuf,u8 len)
{
    u32 num;
	if(len==2)num=(pbuf[1]<<8)|pbuf[0];
	else if(len==4)num=(pbuf[3]<<24)|(pbuf[2]<<16)|(pbuf[1]<<8)|pbuf[0];
	return num;
}




