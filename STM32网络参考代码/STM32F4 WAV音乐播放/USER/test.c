#include <stm32f10x_lib.h>
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h" 
#include "key.h"
#include "lcd.h"	   
#include "dma.h"
#include "mmc_sd.h"
#include "text.h"
#include "fat.h"
#include "fontupd.h"
#include "sysfile.h"
#include "spi.h"
#include "jpegbmp.h"
#include "dac.h"
#include "wavplay.h"
//Mini STM32�����巶������25
//ͼƬ��ʾ ʵ��
//����ԭ��@ALIENTEK
//2010.6.18
u32 sd_Capacity;
u8 volume;			 					
int main(void)
{	 		 
	u8 key;
	SD_Error i;		  
	FileInfoStruct *FileInfo;	   		 
	u16 pic_cnt=0;//��ǰĿ¼��ͼƬ�ļ��ĸ���
	u16 index=0;  //��ǰѡ����ļ����	   
	u16 time=0;    	     	  					   

	Stm32_Clock_Init(9);//ϵͳʱ������
	delay_init(72);		//��ʱ��ʼ��
	uart_init(72,9600); //����1��ʼ��  	  
	LCD_Init();			//��ʼ��Һ��		  
	KEY_Init();			//������ʼ��
	LED_Init();         //LED��ʼ��
	//SPI_FLASH_Init(); //SPI FLASHʹ�� 	  
	POINT_COLOR=RED;		     
	LCD_ShowString(60,50,"Mini STM32");	 
	MyDAC_Init();
	POINT_COLOR=RED;
	LCD_ShowString(60,50,"Mini STM32");
	i=SD_InitAndConfig();
	while(FAT_Init())//FAT ����
	{
		LCD_ShowString(60,130,"fat wrong");  
		
		if(i!=SD_OK)LCD_ShowString(60,150,"SD wrong!");//SD����ʼ��ʧ�� 
			  
		delay_ms(500);
		LCD_ShowNum(120,170,FAT_Init(),2,16);
		delay_ms(500);
		LED1=!LED1;	   
	}	   				 
	while(SysInfoGet(1))//�õ������ļ���  
	{
		LCD_ShowString(60,130,"can't find file");  
		delay_ms(500);  
		FAT_Init();
		SD_Init();
		LED1=!LED1;
		LCD_Fill(60,130,240,170,WHITE);//�����ʾ			  
		delay_ms(500);		
	}
	LCD_ShowString(60,130,"start to play"); 
	delay_ms(1000);
	Cur_Dir_Cluster=PICCLUSTER;
	volume=35;//Ԥ������
	while(1)
	{	    			 
		pic_cnt=0;	 
		Get_File_Info(Cur_Dir_Cluster,FileInfo,T_WAV,&pic_cnt);//��ȡ��ǰ�ļ��������Ŀ���ļ����� 		    
		if(pic_cnt==0)//û��WAV�ļ�
		{
			LCD_Clear(WHITE);//����	   
			while(1)
			{	  
				if(time%2==0)LCD_ShowString(32,150,"No WAV file");		 
				else LCD_Clear(WHITE);
				time++;
				delay_ms(300);
			}
		}								   
		FileInfo=&F_Info[0];//�����ݴ�ռ�.
		index=1;
		while(1)
		{
			Get_File_Info(Cur_Dir_Cluster,FileInfo,T_WAV,&index);//�õ��ļ���Ϣ	 
			LCD_Clear(WHITE);//����,������һ��ͼƬ��ʱ��,һ������
			POINT_COLOR=RED; 				
			LCD_ShowString(60,10,FileInfo->F_Name);//��ʾ�ļ�����
			Playwav(FileInfo);//��ʼ����			     			
			key=KEY_Scan();
			if(key==1)break;//��һ��
			else if(key==2)//��һ��
			{
				if(index>1)index-=2;
				else index=pic_cnt-1;
				break;
			}
			delay_ms(500);
			index++;
			if(index>pic_cnt)index=1;//���ŵ�һ��	  	 		 
		}
	}			   		 			  
}		 





