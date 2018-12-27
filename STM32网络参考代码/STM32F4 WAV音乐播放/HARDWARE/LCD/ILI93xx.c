#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "usart.h"
#include "fsmc.h"

//������ɫ,������ɫ
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

typedef struct
{
  vu16 LCD_REG;
  vu16 LCD_RAM;
} LCD_TypeDef;

// LCD /CS is CE4 - Bank 4 of NOR/SRAM Bank 1~4
#define LCD_BASE        ((u32)(0x60000000 | 0x0C000000))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
 
//д�Ĵ���
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
  	LCD->LCD_REG = LCD_Reg;
  	LCD->LCD_RAM = LCD_RegValue;	    		 
}	   
//���Ĵ���
u16 LCD_ReadReg(u8 LCD_Reg)
{										   
  	LCD->LCD_REG = LCD_Reg;
  	return (LCD->LCD_RAM);
}   
//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
  	LCD->LCD_REG = R34;
}	 
//LCDдGRAM
void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD->LCD_RAM = RGB_Code;//дʮ��λGRAM
}
//LCD��GRAM
u16 LCD_ReadRAM(void)
{
  LCD->LCD_REG = R34; //ѡ��GRAM�Ĵ���
  return LCD->LCD_RAM;
}
//��ILI93xx����������ΪGBR��ʽ��������д���ʱ��ΪRGB��ʽ��
//ͨ���ú���ת��
//c:GBR��ʽ����ɫֵ
//����ֵ��RGB��ʽ����ɫֵ
u16 LCD_BGR2RGB(u16 c)
{
  u16  r,g,b,rgb;   
  b=(c>>0)&0x1f;
  g=(c>>5)&0x3f;
  r=(c>>11)&0x1f;	 
  rgb=(b<<11)+(g<<5)+(r<<0);		 
  return(rgb);
}		 
//��ȡ��ĳ�����ɫֵ	 
//x:0~239
//y:0~319
//����ֵ:�˵����ɫ
u16 LCD_ReadPoint(u16 x,u16 y)
{
  	LCD_SetCursor(x,y);
  	if(DeviceCode==0x7783||DeviceCode==0x4531)
  	return (LCD_ReadRAM());
  	else
  	return (LCD_BGR2RGB(LCD_ReadRAM()));
}
//LCD������ʾ
void LCD_DisplayOn(void)
{					   
	LCD_WriteReg(R7, 0x0173); //26��ɫ��ʾ����
}	 
//LCD�ر���ʾ
void LCD_DisplayOff(void)
{	   
	LCD_WriteReg(R7, 0x0);//�ر���ʾ 
}    
//LCD��ʱ���� 10MS
void Delay (u32 nCount)
{
	volatile int i;	 	
	for (i=0;i<nCount*100;i++);
}
//���ù��λ��
//Xpos:������
//Ypos:������
__inline void LCD_SetCursor(u8 Xpos, u16 Ypos)
{
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);
} 
//����
//x:0~239
//y:0~319
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(u16 x,u16 y)
{
  	LCD_SetCursor(x,y);//���ù��λ��
	LCD->LCD_REG=R34;//��ʼд��GRAM
  	LCD->LCD_RAM=POINT_COLOR; 
} 	 
//��ʼ��lcd
//�ó�ʼ���������Գ�ʼ������ILI93XXҺ��,�������������ǻ���ILI9320��!!!
//�������ͺŵ�����оƬ��û�в���! 
void LCD_Init(void)
{ 
	u32 i;
  	LCD_LineCfg();
	LCD_FSMCCfg();	  					 
	Delay(5); // delay 50 ms 
	LCD_WriteReg(0x0000,0x0001);
	Delay(5); // delay 50 ms 
	DeviceCode = LCD_ReadReg(0x0000);   
//	printf("ID:%d\n",DeviceCode);   
	if(DeviceCode==0x9325||DeviceCode==0x9328)
	{
  		LCD_WriteReg(0x00e7,0x0010);      
        LCD_WriteReg(0x0000,0x0001);  			//start internal osc
        LCD_WriteReg(0x0001,0x0100);     
        LCD_WriteReg(0x0002,0x0700); 				//power on sequence                     
        LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4) ); 	//65K 
        LCD_WriteReg(0x0004,0x0000);                                   
        LCD_WriteReg(0x0008,0x0207);	           
        LCD_WriteReg(0x0009,0x0000);         
        LCD_WriteReg(0x000a,0x0000); 				//display setting         
        LCD_WriteReg(0x000c,0x0001);				//display setting          
        LCD_WriteReg(0x000d,0x0000); 				//0f3c          
        LCD_WriteReg(0x000f,0x0000);
//Power On sequence //
        LCD_WriteReg(0x0010,0x0000);   
        LCD_WriteReg(0x0011,0x0007);
        LCD_WriteReg(0x0012,0x0000);                                                                 
        LCD_WriteReg(0x0013,0x0000);                 
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0010,0x1590);   
        LCD_WriteReg(0x0011,0x0227);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0012,0x009c);                  
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0013,0x1900);   
        LCD_WriteReg(0x0029,0x0023);
        LCD_WriteReg(0x002b,0x000e);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x0000);           
///////////////////////////////////////////////////////      
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0030,0x0007); 
        LCD_WriteReg(0x0031,0x0707);   
        LCD_WriteReg(0x0032,0x0006);
        LCD_WriteReg(0x0035,0x0704);
        LCD_WriteReg(0x0036,0x1f04); 
        LCD_WriteReg(0x0037,0x0004);
        LCD_WriteReg(0x0038,0x0000);        
        LCD_WriteReg(0x0039,0x0706);     
        LCD_WriteReg(0x003c,0x0701);
        LCD_WriteReg(0x003d,0x000f);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0050,0x0000);        
        LCD_WriteReg(0x0051,0x00ef);   
        LCD_WriteReg(0x0052,0x0000);     
        LCD_WriteReg(0x0053,0x013f);
        LCD_WriteReg(0x0060,0xa700);        
        LCD_WriteReg(0x0061,0x0001); 
        LCD_WriteReg(0x006a,0x0000);
        LCD_WriteReg(0x0080,0x0000);
        LCD_WriteReg(0x0081,0x0000);
        LCD_WriteReg(0x0082,0x0000);
        LCD_WriteReg(0x0083,0x0000);
        LCD_WriteReg(0x0084,0x0000);
        LCD_WriteReg(0x0085,0x0000);
      
        LCD_WriteReg(0x0090,0x0010);     
        LCD_WriteReg(0x0092,0x0000);  
        LCD_WriteReg(0x0093,0x0003);
        LCD_WriteReg(0x0095,0x0110);
        LCD_WriteReg(0x0097,0x0000);        
        LCD_WriteReg(0x0098,0x0000);  
         //display on sequence     
        LCD_WriteReg(0x0007,0x0133);
    
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x0000);
	}
	else if(DeviceCode==0x9320||DeviceCode==0x9300)
	{
		LCD_WriteReg(0x00,0x0000);
		LCD_WriteReg(0x01,0x0100);	//Driver Output Contral.
		LCD_WriteReg(0x02,0x0700);	//LCD Driver Waveform Contral.
//		LCD_WriteReg(0x03,0x1030);	//Entry Mode Set.
		LCD_WriteReg(0x03,0x1018);	//Entry Mode Set.
	
		LCD_WriteReg(0x04,0x0000);	//Scalling Contral.
		LCD_WriteReg(0x08,0x0202);	//Display Contral 2.(0x0207)
		LCD_WriteReg(0x09,0x0000);	//Display Contral 3.(0x0000)
		LCD_WriteReg(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
		LCD_WriteReg(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
		LCD_WriteReg(0x0d,0x0000);	//Frame Maker Position.
		LCD_WriteReg(0x0f,0x0000);	//Extern Display Interface Contral 2.
	
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
		LCD_WriteReg(0x07,0x0101);	//Display Contral.
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
	
		LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
		LCD_WriteReg(0x11,0x0007);								//Power Control 2.(0x0001)
		LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
		LCD_WriteReg(0x13,0x0b00);								//Power Control 4.
		LCD_WriteReg(0x29,0x0000);								//Power Control 7.
	
		LCD_WriteReg(0x2b,(1<<14)|(1<<4));
		
		LCD_WriteReg(0x50,0);		//Set X Start.
		LCD_WriteReg(0x51,239);	//Set X End.
		LCD_WriteReg(0x52,0);		//Set Y Start.
		LCD_WriteReg(0x53,319);	//Set Y End.
	
		LCD_WriteReg(0x60,0x2700);	//Driver Output Control.
		LCD_WriteReg(0x61,0x0001);	//Driver Output Control.
		LCD_WriteReg(0x6a,0x0000);	//Vertical Srcoll Control.
	
		LCD_WriteReg(0x80,0x0000);	//Display Position? Partial Display 1.
		LCD_WriteReg(0x81,0x0000);	//RAM Address Start? Partial Display 1.
		LCD_WriteReg(0x82,0x0000);	//RAM Address End-Partial Display 1.
		LCD_WriteReg(0x83,0x0000);	//Displsy Position? Partial Display 2.
		LCD_WriteReg(0x84,0x0000);	//RAM Address Start? Partial Display 2.
		LCD_WriteReg(0x85,0x0000);	//RAM Address End? Partial Display 2.
	
		LCD_WriteReg(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
		LCD_WriteReg(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
		LCD_WriteReg(0x93,0x0001);	//Panel Interface Contral 3.
		LCD_WriteReg(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
		LCD_WriteReg(0x97,(0<<8));	//
		LCD_WriteReg(0x98,0x0000);	//Frame Cycle Contral.

	
		LCD_WriteReg(0x07,0x0173);	//(0x0173)
	}
	else if(DeviceCode==0x4531)
	{		
		// Setup display
		LCD_WriteReg(0x00,0x0001);
	    LCD_WriteReg(0x10,0x0628);
	    LCD_WriteReg(0x12,0x0006);
	    LCD_WriteReg(0x13,0x0A32);
	    LCD_WriteReg(0x11,0x0040);
	    LCD_WriteReg(0x15,0x0050);
	    LCD_WriteReg(0x12,0x0016);
	    Delay(15);
	    LCD_WriteReg(0x10,0x5660);
	    Delay(15);
	    LCD_WriteReg(0x13,0x2A4E);
	    LCD_WriteReg(0x01,0x0100);
	    LCD_WriteReg(0x02,0x0300);
	
	    LCD_WriteReg(0x03,0x1030);
//	    LCD_WriteReg(0x03,0x1038);
	
	    LCD_WriteReg(0x08,0x0202);
	    LCD_WriteReg(0x0A,0x0000);
	    LCD_WriteReg(0x30,0x0000);
	    LCD_WriteReg(0x31,0x0402);
	    LCD_WriteReg(0x32,0x0106);
	    LCD_WriteReg(0x33,0x0700);
	    LCD_WriteReg(0x34,0x0104);
	    LCD_WriteReg(0x35,0x0301);
	    LCD_WriteReg(0x36,0x0707);
	    LCD_WriteReg(0x37,0x0305);
	    LCD_WriteReg(0x38,0x0208);
	    LCD_WriteReg(0x39,0x0F0B);
	    Delay(15);
	    LCD_WriteReg(0x41,0x0002);
	    LCD_WriteReg(0x60,0x2700);
	    LCD_WriteReg(0x61,0x0001);
	    LCD_WriteReg(0x90,0x0119);
	    LCD_WriteReg(0x92,0x010A);
	    LCD_WriteReg(0x93,0x0004);
	    LCD_WriteReg(0xA0,0x0100);
//	    LCD_WriteReg(0x07,0x0001);
	    Delay(15);
//	    LCD_WriteReg(0x07,0x0021); 
	    Delay(15);
//	    LCD_WriteReg(0x07,0x0023);
	    Delay(15);
//	    LCD_WriteReg(0x07,0x0033);
	    Delay(15);
	    LCD_WriteReg(0x07,0x0133);
	    Delay(15);
	    LCD_WriteReg(0xA0,0x0000);
	    Delay(20);
	} 							 
  	for(i=50000;i>0;i--);
	LCD_Clear(WHITE);
}  		  
  
//��������
//Color:Ҫ���������ɫ
void LCD_Clear(u16 Color)
{
	u32 index=0;      
	LCD_SetCursor(0x00,0x0000);//���ù��λ�� 
	LCD_WriteRAM_Prepare();     //��ʼд��GRAM	 	  
	for(index=0;index<76800;index++)
	{
		LCD->LCD_RAM = Color;    
	}
}  
//��ָ�����������ָ����ɫ
//�����С:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(u8 xsta,u16 ysta,u8 xend,u16 yend,u16 color)
{                    
    u32 n;
	//���ô���										
	LCD_WriteReg(R80, xsta); //ˮƽ����GRAM��ʼ��ַ
	LCD_WriteReg(R81, xend); //ˮƽ����GRAM������ַ
	LCD_WriteReg(R82, ysta); //��ֱ����GRAM��ʼ��ַ
	LCD_WriteReg(R83, yend); //��ֱ����GRAM������ַ	
	LCD_SetCursor(xsta,ysta);//���ù��λ��  
	LCD_WriteRAM_Prepare();  //��ʼд��GRAM	 	   	   
	n=(u32)(yend-ysta+1)*(xend-xsta+1);    
	while(n--){LCD->LCD_RAM = color;}//��ʾ��������ɫ. 
	//�ָ�����
	LCD_WriteReg(R80, 0x0000); //ˮƽ����GRAM��ʼ��ַ
	LCD_WriteReg(R81, 0x00EF); //ˮƽ����GRAM������ַ
	LCD_WriteReg(R82, 0x0000); //��ֱ����GRAM��ʼ��ַ
	LCD_WriteReg(R83, 0x013F); //��ֱ����GRAM������ַ	    
}  
//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_DrawPoint(uRow,uCol);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    
//������
void LCD_DrawRectangle(u8 x1, u16 y1, u8 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void Draw_Circle(u8 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 
//��ָ��λ����ʾһ���ַ�
//x:0~234
//y:0~308
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(u8 x,u16 y,u8 num,u8 size,u8 mode)
{       
#define MAX_CHAR_POSX 232
#define MAX_CHAR_POSY 304 
    u8 temp;
    u8 pos,t;      
    if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;	    
	//���ô���										
	LCD_WriteReg(R80,x);           //ˮƽ����GRAM��ʼ��ַ
	LCD_WriteReg(R81,x+(size/2-1));//ˮƽ����GRAM������ַ
	LCD_WriteReg(R82,y);           //��ֱ����GRAM��ʼ��ַ
	LCD_WriteReg(R83,y+size-1);    //��ֱ����GRAM������ַ	
	LCD_SetCursor(x,y);            //���ù��λ��  
	LCD_WriteRAM_Prepare();        //��ʼд��GRAM	   
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	if(!mode) //�ǵ��ӷ�ʽ
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//����1206����
			else temp=asc2_1608[num][pos];		 //����1608����
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)
				{
					LCD->LCD_RAM=POINT_COLOR;
				}else LCD->LCD_RAM=BACK_COLOR;	        
		        temp>>=1; 
		    }
		}	
	}else//���ӷ�ʽ
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//����1206����
			else temp=asc2_1608[num][pos];		 //����1608����
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//��һ����     
		        temp>>=1; 
		    }
		}
	}	    
	//�ָ������С	 
	LCD_WriteReg(R80, 0x0000); //ˮƽ����GRAM��ʼ��ַ
	LCD_WriteReg(R81, 0x00EF); //ˮƽ����GRAM������ַ
	LCD_WriteReg(R82, 0x0000); //��ֱ����GRAM��ʼ��ַ
	LCD_WriteReg(R83, 0x013F); //��ֱ����GRAM������ַ
}  
//m^n����
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//color:��ɫ
//num:��ֵ(0~4294967295);	 
void LCD_ShowNum(u8 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
//��ʾ2������
//x,y:�������
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~99);	 
void LCD_Show2Num(u8 x,u16 y,u16 num,u8 len,u8 size,u8 mode)
{         	
	u8 t,temp;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode); 
	}
} 
//��ʾ�ַ���
//x,y:�������  
//*p:�ַ�����ʼ��ַ
//��16����
void LCD_ShowString(u8 x,u16 y,const u8 *p)
{         
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(WHITE);}
        LCD_ShowChar(x,y,*p,16,0);
        x+=8;
        p++;
    }  
}

void LCD_LineCfg(void)
{	   	 
	RCC->AHBENR|=1<<8;	   //ʹ��FSMC
	RCC->APB2ENR|=1<<0;    //ʹ��AFIOʱ��
	RCC->APB2ENR|=1<<5;    //ʹ��PORTDʱ��
	RCC->APB2ENR|=1<<6;    //ʹ��PORTEʱ��
	RCC->APB2ENR|=1<<7;    //ʹ��PORTFʱ��
	RCC->APB2ENR|=1<<8;    //ʹ��PORTGʱ��
 
	GPIOD->CRL&=0XFF00FF00; 
	GPIOD->CRL|=0X00BB00BB;//PD0,PD1,PD4,PD5������� 	 
	GPIOD->CRH&=0X00FFF000; 
	GPIOD->CRH|=0XBB000BBB;//PD8~PD10,PD14,PD15 �������   	 
    GPIOD->ODR|=0XC733;      //�����

   	GPIOE->CRL&=0X00FFFFFF; 
	GPIOE->CRL|=0XB3000000;//PE7 �������   	 
	GPIOE->CRH=0XBBBBBBBB;//PE8~PE15�������   	 
    GPIOE->ODR|=0XFFC0;      //�����

	GPIOF->CRL&=0XFFFFFFF0;    
	GPIOF->CRL|=0X0000000B;//PF0 �������   	   	 
    GPIOF->ODR|=1;      //�����

	GPIOG->CRH&=0XFFF0FFFF;    
	GPIOG->CRH|=0X000B0000;//PG12 �������   	   	 
    GPIOG->ODR|=1<<12;      //�����
}
























