#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "24cxx.h"
#include "flash.h"
#include "flash.h"
#include "spi.h"	  

u8 state=0;
void beepms(u16 va);
void xianshi(void);//��ʾ��Ϣ  
void refshow(void);//ˢ����ʾ

void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);//����   
 	POINT_COLOR=BLUE;//��������Ϊ��ɫ 
	LCD_ShowString(lcddev.width-24,0,200,16,16,"RST");//��ʾ��������
  	POINT_COLOR=RED;//���û�����ɫ 
}
void xianshi()//��ʾ��Ϣ
{ 
	BACK_COLOR=WHITE;
	POINT_COLOR=RED;   
	//��ʾ32*32����
	showhanzi32(0,0,0);	 //��
	showhanzi32(40,0,1);	 //��
	showhanzi32(80,0,2);    //��
	//��ʾ16*16����
	showhanzi16(0,35,0);	  //ר
	showhanzi16(20,35,1);	  //ע
	showhanzi16(40,35,2);	  //��
	showhanzi16(60,35,3);	  //ʾ
	showhanzi16(80,35,4);	  //��
	showhanzi16(100,35,5);	  //��	   
	LCD_ShowString(0,55,200,16,16,"1.8 TFT SPI");
}
void showqq()
{ 
	u16 x,y; 
	x=0;
	y=75;
	while(y<lcddev.height-39)
	{
		x=0;
		while(x<lcddev.width-39)
		{
			showimage(x,y);	
			x+=40;
		}
		y+=40;
	 }	  
}
void refshow(void)	 //ˢ����ʾ
{
	switch(state)
	{
		case 0:
		LCD_Clear(WHITE);
	    xianshi();
    	showqq();
		break;
		case 1:
		LCD_Clear(BLACK);	
		break;
		case 2:
		LCD_Clear(RED);
		break;
		case 3:
		LCD_Clear(GREEN);
		break;
		case 4:
		LCD_Clear(BLUE);
		break;
	}	
}
void jiance(u8 key) //��ⰴ��
{ 	
	if(key==2)
	{
		state++;
		LCD_Init();
		if(state==5)
		{
			state=0; 
		}
		refshow();//ˢ����ʾ
	}

} 
void beepms(u16 va)
{
	beep=1;
	delay_ms(va);
    beep=0;
}				
int main(void)
 {	 
	u16 i=0;	 
	u8 key=0;	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
 	LED_Init();			     //LED�˿ڳ�ʼ��
    SPI_Flash_Init();//��ʼ��Flash;
	KEY_Init();	 //������ʼ��	   	 
	 	
	SPI1_Init();	//SPI1��ʼ��
	LCD_Init();	  			
 	POINT_COLOR=RED;//��������Ϊ��ɫ 
	xianshi();	   //��ʾ��Ϣ
	showqq();	   //��ʾQQ
	while(1)
	{ 
	   key=KEY_Scan(0);//ɨ�谴��	 
	   jiance(key);//��ⰴ��  

		i++;
		if(i==65000)
		{
			i=0;
			LED0=!LED0;
		}
	}
}


