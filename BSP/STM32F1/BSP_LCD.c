#include "BSP_LCD.h"

#define LL_Lib 0   //LL��
#define HAL_Lib 1  //HAL��
#define RTOS_Lib 0 //FreeRTOS

#if RTOS_Lib
#include "cmsis_os.h"
#endif

//LCD�Ļ�����ɫ�ͱ���ɫ
uint16_t POINT_COLOR = 0x0000; //������ɫ
uint16_t BACK_COLOR = 0xFFFF;  //����ɫ

void SPI_WriteByte(uint8_t Data)
{
	uint8_t i = 0;
	for (i = 8; i > 0; i--)
	{
		if (Data & 0x80)
			SPILCD_SDA_SET; //�������
		else
			SPILCD_SDA_RESET;

		SPILCD_SCK_RESET;
		SPILCD_SCK_SET;
		Data <<= 1;
	}
}

//д�Ĵ�������
//regval:�Ĵ���ֵ
void LCD_WR_REG(uint8_t regval)
{
	SPILCD_CS_RESET; //LCD_CS=0
	SPILCD_RS_RESET;
	SPI_WriteByte(regval);
	SPILCD_CS_SET; //LCD_CS=1
}
//дLCD����
//data:Ҫд���ֵ
void LCD_WR_DATA(uint16_t data)
{
	SPILCD_CS_RESET; //LCD_CS=0
	SPILCD_RS_SET;
	SPI_WriteByte(data >> 8);
	SPI_WriteByte(data);
	SPILCD_CS_SET; //LCD_CS=1
}
void LCD_WR_DATA8(uint8_t da) //д8λ����
{
	SPILCD_CS_RESET; //LCD_CS=0
	SPILCD_RS_SET;
	SPI_WriteByte(da);
	SPILCD_CS_SET; //LCD_CS=1
}

void LCD_WR_DATA16(uint16_t da) //д16λ����
{
	SPILCD_CS_RESET; //LCD_CS=0
	SPILCD_RS_SET;
	SPI_WriteByte(da >> 8);
	SPI_WriteByte(da);
	SPILCD_CS_SET; //LCD_CS=1
}

//д�Ĵ���
//LCD_Reg:�Ĵ�����ַ
//LCD_RegValue:Ҫд�������
void LCD_WR_REG_DATA(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*************************************************
��������LCD_Set_Region
���ܣ�����lcd��ʾ�����ڴ�����д�������Զ�����
��ڲ�����xy�����յ�
����ֵ����
*************************************************/
void Lcd_SetRegion(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
	LCD_WR_REG(0x2a);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(x_start);
	//	LCD_WR_DATA8(x_start+2);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(x_end);
	//	LCD_WR_DATA8(x_end+2);

	LCD_WR_REG(0x2b);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(y_start);
	//	LCD_WR_DATA8(y_start+1);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(y_end);
	//	LCD_WR_DATA8(y_end+1);

	LCD_WR_REG(0x2c);
}

//���ù��λ��
//Xpos:������
//Ypos:������
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	Lcd_SetRegion(Xpos, Ypos, Xpos, Ypos);
}

//����
//x,y:����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t Point_Color)
{
	LCD_SetCursor(x, y); //���ù��λ��
	LCD_WR_DATA(POINT_COLOR);
}
//��ʼ��lcd
void LCD_Init(void)
{
	SPILCD_RST_RESET; //LCD_RST=0	 //SPI�ӿڸ�λ

#if LL_Lib
	LL_mDelay(100);
#endif
#if HAL_Lib
	HAL_Delay(100);
#endif
#if RTOS_Lib
	osDelay(100);
#endif

	SPILCD_RST_SET; //LCD_RST=1
#if LL_Lib
	LL_mDelay(50);
#endif
#if HAL_Lib
	HAL_Delay(50);
#endif
#if RTOS_Lib
	osDelay(50);
#endif

	LCD_WR_REG(0x11); //Sleep out
#if LL_Lib
	LL_mDelay(120);
#endif
#if HAL_Lib
	HAL_Delay(120);
#endif
#if RTOS_Lib
	osDelay(120);
#endif
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);

	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	//------------------------------------End ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB4); //Dot inversion
	LCD_WR_DATA8(0x03);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x04);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0XC0);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x2A);

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);
	//---------------------------------End ST7735S Power Sequence-------------------------------------//
	LCD_WR_REG(0xC5); //VCOM
	LCD_WR_DATA8(0x1A);

	LCD_WR_REG(0x36); //MX, MY, RGB mode
	LCD_WR_DATA8(0xC0);
	//------------------------------------ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x22);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2A);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);
	//------------------------------------End ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0x3A); //65k mode
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0x29); //Display on
	// LCD_Clear(YELLOW);
	// LCD_Fill(0,0,128,160,BLUE);
}
//��������
//color:Ҫ���������ɫ
void LCD_Clear(uint16_t color)
{
	unsigned int i, m;
	Lcd_SetRegion(0, 0, 128 - 1, 160 - 1);
	LCD_WR_REG(0x2C);
	for (i = 0; i < 128; i++)
		for (m = 0; m < 160; m++)
		{
			LCD_WR_DATA16(color);
		}
}
//��ָ����������䵥����ɫ
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)
//color:Ҫ������ɫ
//void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
//{
//	uint16_t i,j;
//	uint16_t xlen=0;
//	xlen=ex-sx+1;
//	for(i=sy;i<=ey;i++)
//	{
//	 LCD_SetCursor(sx,i);      				//���ù��λ��
//		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM
//		for(j=0;j<xlen;j++)LCD_WR_DATA(color);	//���ù��λ��
//	}
//}
//��ָ�����������ָ����ɫ��
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)
//color:Ҫ������ɫ
//void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
//{
//	uint16_t height,width;
//	uint16_t i,j;
//	width=ex-sx+1; 		//�õ����Ŀ��
//	height=ey-sy+1;		//�߶�
// 	for(i=0;i<height;i++)
//	{
// 		LCD_SetCursor(sx,sy+i);   	//���ù��λ��
//		LCD_WriteRAM_Prepare();     //��ʼд��GRAM
//		for(j=0;j<width;j++)LCD->LCD_RAM=color[i*height+j];//д������
//	}
//}
//����
//x1,y1:�������
//x2,y2:�յ�����
//void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
//{
//	uint16_t t;
//	int xerr=0,yerr=0,delta_x,delta_y,distance;
//	int incx,incy,uRow,uCol;
//	delta_x=x2-x1; //������������
//	delta_y=y2-y1;
//	uRow=x1;
//	uCol=y1;
//	if(delta_x>0)incx=1; //���õ�������
//	else if(delta_x==0)incx=0;//��ֱ��
//	else {incx=-1;delta_x=-delta_x;}
//	if(delta_y>0)incy=1;
//	else if(delta_y==0)incy=0;//ˮƽ��
//	else{incy=-1;delta_y=-delta_y;}
//	if( delta_x>delta_y)distance=delta_x; //ѡȡ��������������
//	else distance=delta_y;
//	for(t=0;t<=distance+1;t++ )//�������
//	{
//		LCD_DrawPoint(uRow,uCol);//����
//		xerr+=delta_x ;
//		yerr+=delta_y ;
//		if(xerr>distance)
//		{
//			xerr-=distance;
//			uRow+=incx;
//		}
//		if(yerr>distance)
//		{
//			yerr-=distance;
//			uCol+=incy;
//		}
//	}
//}
//������
//(x1,y1),(x2,y2):���εĶԽ�����
//void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
//{
//	LCD_DrawLine(x1,y1,x2,y1);
//	LCD_DrawLine(x1,y1,x1,y2);
//	LCD_DrawLine(x1,y2,x2,y2);
//	LCD_DrawLine(x2,y1,x2,y2);
//}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
//void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
//{
//	int a,b;
//	int di;
//	a=0;b=r;
//	di=3-(r<<1);             //�ж��¸���λ�õı�־
//	while(a<=b)
//	{
//		LCD_DrawPoint(x0+a,y0-b);             //5
// 	LCD_DrawPoint(x0+b,y0-a);             //0
//		LCD_DrawPoint(x0+b,y0+a);             //4
//		LCD_DrawPoint(x0+a,y0+b);             //6
//		LCD_DrawPoint(x0-a,y0+b);             //1
// 	LCD_DrawPoint(x0-b,y0+a);
//		LCD_DrawPoint(x0-a,y0-b);             //2
//  LCD_DrawPoint(x0-b,y0-a);             //7
//		a++;
//		//ʹ��Bresenham�㷨��Բ
//		if(di<0)di +=4*a+6;
//		else
//		{
//			di+=10+4*(a-b);
//			b--;
//		}
//	}
//}
////��ָ��λ����ʾһ������(16*16��С)
//void showhanzi16(unsigned int x,unsigned int y,unsigned char index)
//{
//	unsigned char i,j,k;
//	const unsigned char *temp=hanzi16;
//	temp+=index*32;
//	for(j=0;j<16;j++)
//	{
//		LCD_SetCursor(x,y+j);
//		LCD_WriteRAM_Prepare();	//��ʼд��GRAM
//		for(k=0;k<2;k++)
//		{
//			for(i=0;i<8;i++)
//			{
//			 	if((*temp&(1<<i))!=0)
//				{
//					LCD_WR_DATA(POINT_COLOR);
//				}
//				else
//				{
//					LCD_WR_DATA(BACK_COLOR);
//				}
//			}
//			temp++;
//		}
//	 }
//}
////��ָ��λ����ʾһ������(32*32��С)
//void showhanzi32(unsigned int x,unsigned int y,unsigned char index)
//{
//	unsigned char i,j,k;
//	const unsigned char *temp=hanzi32;
//	temp+=index*128;
//	for(j=0;j<32;j++)
//	{
//		LCD_SetCursor(x,y+j);
//		LCD_WriteRAM_Prepare();	//��ʼд��GRAM
//		for(k=0;k<4;k++)
//		{
//			for(i=0;i<8;i++)
//			{
//			 	if((*temp&(1<<i))!=0)
//				{
//					LCD_WR_DATA(POINT_COLOR);
//				}
//				else
//				{
//					LCD_WR_DATA(BACK_COLOR);
//				}
//			}
//			temp++;
//		}
//	 }
//}
//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
//{
//    uint8_t temp,t1,t;
//	uint16_t y0=y;
//	uint16_t colortemp=POINT_COLOR;
//	//���ô���
//	num=num-' ';//�õ�ƫ�ƺ��ֵ
//	if(!mode) //�ǵ��ӷ�ʽ
//	{
//	    for(t=0;t<size;t++)
//	    {
//			if(size==12)temp=asc2_1206[num][t];  //����1206����
//			else temp=asc2_1608[num][t];		 //����1608����
//	        for(t1=0;t1<8;t1++)
//			{
//		        if(temp&0x80)POINT_COLOR=colortemp;
//				else POINT_COLOR=BACK_COLOR;
//				LCD_DrawPoint(x,y);
//				temp<<=1;
//				y++;
//				if(y>=lcddev.height){POINT_COLOR=colortemp;return;}//��������
//				if((y-y0)==size)
//				{
//					y=y0;
//					x++;
//					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//��������
//					break;
//				}
//			}
//	    }
//	}else//���ӷ�ʽ
//	{
//	    for(t=0;t<size;t++)
//	    {
//			if(size==12)temp=asc2_1206[num][t];  //����1206����
//			else temp=asc2_1608[num][t];		 //����1608����
//	        for(t1=0;t1<8;t1++)
//			{
//		        if(temp&0x80)LCD_DrawPoint(x,y);
//				temp<<=1;
//				y++;
//				if(y>=lcddev.height){POINT_COLOR=colortemp;return;}//��������
//				if((y-y0)==size)
//				{
//					y=y0;
//					x++;
//					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//��������
//					break;
//				}
//			}
//	    }
//	}
//	POINT_COLOR=colortemp;
//}
//m^n����
//����ֵ:m^n�η�.
//uint32_t LCD_Pow(uint8_t m,uint8_t n)
//{
//	uint32_t result=1;
//	while(n--)result*=m;
//	return result;
//}
//��ʾ����,��λΪ0,����ʾ
//x,y :�������
//len :���ֵ�λ��
//size:�����С
//color:��ɫ
//num:��ֵ(0~4294967295);
//void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size)
//{
//	uint8_t t,temp;
//	uint8_t enshow=0;
//	for(t=0;t<len;t++)
//	{
//		temp=(num/LCD_Pow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
//				continue;
//			}else enshow=1;
//
//		}
//	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0);
//	}
//}
//��ʾ����,��λΪ0,������ʾ
//x,y:�������
//num:��ֵ(0~999999999);
//len:����(��Ҫ��ʾ��λ��)
//size:�����С
//mode:
//[7]:0,�����;1,���0.
//[6:1]:����
//[0]:0,�ǵ�����ʾ;1,������ʾ.
//void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode)
//{
//	uint8_t t,temp;
//	uint8_t enshow=0;
//	for(t=0;t<len;t++)
//	{
//		temp=(num/LCD_Pow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);
//				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);
// 				continue;
//			}else enshow=1;
//
//		}
//	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01);
//	}
//}
//��ʾ�ַ���
//x,y:�������
//width,height:�����С
//size:�����С
//*p:�ַ�����ʼ��ַ
//void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)
//{
//	uint8_t x0=x;
//	width+=x;
//	height+=y;
//    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
//    {
//        if(x>=width){x=x0;y+=size;}
//        if(y>=height)break;//�˳�
//        LCD_ShowChar(x,y,*p,size,1);
//        x+=size/2;
//        p++;
//    }
//}

//void showimage(uint16_t x,uint16_t y) //��ʾ40*40ͼƬ
//{
//	uint16_t i,j,k;
//	uint16_t da;
//	k=0;
//	for(i=0;i<40;i++)
//	{
//		LCD_SetCursor(x,y+i);
//		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM
//		for(j=0;j<40;j++)
//		{
//			da=qqimage[k*2+1];
//			da<<=8;
//			da|=qqimage[k*2];
//			LCD_WR_DATA(da);
//			k++;
//		}
//	}
//}
