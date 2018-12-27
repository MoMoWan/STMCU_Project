#include "BSP_LCD.h"

static void LCD_Rst(void)
{
	HAL_Delay(100);
	Lcd_RST_Set;
	HAL_Delay(100);
	Lcd_RST_Reset;
	HAL_Delay(100);
	Lcd_RST_Set;
	HAL_Delay(100);
}

void WriteComm(uint16_t CMD)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = CMD;
}

void WriteData(uint16_t tem_data)
{
	*(__IO uint16_t *)(Bank1_LCD_D) = tem_data;
}

/**********************************************
Lcd��ʼ������
***********************************************/
void Lcd_Initialize(void)
{

	LCD_Rst();

	LCD_WR_REG(0x11, 0x2004); //Power control 2
	LCD_WR_REG(0x13, 0xCC00); //Power control 4
	LCD_WR_REG(0x15, 0x2600); //Power control 6
	LCD_WR_REG(0x14, 0x252A); //Power control 5
	//LCD_WR_REG(0x14,0x002A);
	LCD_WR_REG(0x12, 0x0033); //Power control 3
	LCD_WR_REG(0x13, 0xCC04); //Power control 4
	LCD_WR_REG(0x13, 0xCC06); //Power control 4
	LCD_WR_REG(0x13, 0xCC4F); //Power control 4
	LCD_WR_REG(0x13, 0x674F); //Power control 4
	LCD_WR_REG(0x11, 0x2003); //Power control 2
	HAL_Delay(1);
	LCD_WR_REG(0x30, 0x2609); //Gamma control 1
	LCD_WR_REG(0x31, 0x242C); //Gamma control 2
	LCD_WR_REG(0x32, 0x1F23); //Gamma control 3
	LCD_WR_REG(0x33, 0x2425); //Gamma control 4
	LCD_WR_REG(0x34, 0x2226); //Gamma control 5
	LCD_WR_REG(0x35, 0x2523); //Gamma control 6
	LCD_WR_REG(0x36, 0x1C1A); //Gamma control 7
	LCD_WR_REG(0x37, 0x131D); //Gamma control 8
	LCD_WR_REG(0x38, 0x0B11); //Gamma control 9
	LCD_WR_REG(0x39, 0x1210); //Gamma control 10
	LCD_WR_REG(0x3A, 0x1315); //Gamma control 11
	LCD_WR_REG(0x3B, 0x3619); //Gamma control 12
	LCD_WR_REG(0x3C, 0x0D00); //Gamma control 13
	LCD_WR_REG(0x3D, 0x000D); //Gamma control 14
	HAL_Delay(1);
	LCD_WR_REG(0x16, 0x0007); //Power control 7
	LCD_WR_REG(0x02, 0x0013); //LCD-Driving-waveform control
	LCD_WR_REG(0x03, 0x0009); //Entry mode   0x000A
	LCD_WR_REG(0x01, 0x1027); //Driver output control
	HAL_Delay(1);
	LCD_WR_REG(0x08, 0x0303); //Blank period control 1
	LCD_WR_REG(0x0A, 0x000B); //Frame cycle control 1
	LCD_WR_REG(0x0B, 0x0003); //Frame cycle control
	LCD_WR_REG(0x0C, 0x0000); //External interface control  // 18-bit RGB interface (one transfer/pixel)
	LCD_WR_REG(0x41, 0x0000); //Vertical scroll control
	LCD_WR_REG(0x50, 0x0000); //MDDI wake up control
	LCD_WR_REG(0x60, 0x0005); //MTP INIT
	LCD_WR_REG(0x70, 0x000B); //Timing of signal from GOE
	LCD_WR_REG(0x71, 0x0000); //Gate start pulse delay timing
	LCD_WR_REG(0x78, 0x0000); //Vcom Output Control
	LCD_WR_REG(0x7A, 0x0000); //Panel signal control 2
	LCD_WR_REG(0x79, 0x0007); //Panel signal control 1
	LCD_WR_REG(0x07, 0x0051); //Display control
	HAL_Delay(1);
	LCD_WR_REG(0x07, 0x0053); //Display control
	LCD_WR_REG(0x79, 0x0000); //Panel signal control 1
	WriteComm(0x22);
}

/******************************************
��������Lcdд�����
���ܣ���Lcdָ��λ��д��Ӧ�����������
��ڲ�����Index ҪѰַ�ļĴ�����ַ
          ConfigTemp д������ݻ�����ֵ
******************************************/
void LCD_WR_REG(uint16_t Index, uint16_t CongfigTemp)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = Index;
	*(__IO uint16_t *)(Bank1_LCD_D) = CongfigTemp;
}
/************************************************
��������Lcdд��ʼ����
���ܣ�����Lcd�������� ִ��д����
************************************************/
void Lcd_WR_Start(void)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = 0x22;
}
/*************************************************
��������Lcd�����㶨λ����
���ܣ�ָ��320240Һ���ϵ�һ����Ϊд���ݵ���ʼ��
��ڲ�����x ���� 0~239
          y ���� 0~319
����ֵ����
*************************************************/
void Lcd_SetCursor(uint16_t x, uint16_t y)
{
	//�����ת
	LCD_WR_REG(0x20, y); //ˮƽ����
	LCD_WR_REG(0x21, x); //��ֱ����
}

/**********************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ľ�������

ע�⣺xStart��yStart��Xend��Yend������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
          ySrart y�������ʼ��
          Xend   y�������ֹ��
          Yend   y�������ֹ��
����ֵ����
***********************************************/
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend)
{
	//x��y����
	LCD_WR_REG(0x0046, (Yend << 8) | Ystart); //ˮƽGRAM��ֹ����ʼλ��
	LCD_WR_REG(0x0047, Xend);				  //��ֱGRAM��ֹλ��
	LCD_WR_REG(0x0048, Xstart);				  //��ֱ GRAM��ʼλ��

	Lcd_SetCursor(Xstart, Ystart);

	WriteComm(0x022);
}
uint16_t GetPoint(uint16_t x, uint16_t y)
{
	Lcd_SetCursor(x, y);
	WriteComm(0x022);
	x = *(__IO uint16_t *)(Bank1_LCD_D);
	return *(__IO uint16_t *)(Bank1_LCD_D);
}
/**********************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ľ�������

ע�⣺xStart�� yStart������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
          ySrart y�������ֹ��
          xLong Ҫѡ�����ε�x���򳤶�
          yLong  Ҫѡ�����ε�y���򳤶�
����ֵ����
***********************************************/
void Lcd_ColorBox(uint16_t xStart, uint16_t yStart, uint16_t xLong, uint16_t yLong, uint16_t Color)
{
	uint32_t temp;

	BlockWrite(xStart, xStart + xLong - 1, yStart, yStart + yLong - 1);
	for (temp = 0; temp < xLong * yLong; temp++)
	{
		*(__IO uint16_t *)(Bank1_LCD_D) = Color;
	}
}

/******************************************
��������Lcdͼ�����100*100
���ܣ���Lcdָ��λ�����ͼ��
��ڲ�����Index ҪѰַ�ļĴ�����ַ
          ConfigTemp д������ݻ�����ֵ
******************************************/
void LCD_Fill_Pic(uint16_t x, uint16_t y, uint16_t pic_H, uint16_t pic_V, const unsigned char *pic)
{
	unsigned long i;
	unsigned int tmp;

	//Set_address_mode
	LCD_WR_REG(0x01, 0x0127);
	LCD_WR_REG(0x03, 0x000A); //�����������½ǿ�ʼ�������ң����µ���
	BlockWrite(x, x + pic_H - 1, y, y + pic_V - 1);
	for (i = 0; i < pic_H * pic_V * 2; i += 2)
	{
		tmp = pic[i];
		tmp = tmp << 8;
		tmp += pic[i + 1];
		*(__IO uint16_t *)(Bank1_LCD_D) = tmp;
	}
	//Set_address_mode
	LCD_WR_REG(0x01, 0x1027);
	LCD_WR_REG(0x03, 0x010A); //�����������Ͻǿ�ʼ�������ң����ϵ���
}

//��ָ�������ϴ�һ����
void DrawPixel(uint16_t x, uint16_t y, uint16_t Color)
{
	BlockWrite(x, x, y, y);
	*(__IO uint16_t *)(Bank1_LCD_D) = Color;
}

uint16_t LCD_DecToRGB(uint8_t R, uint8_t G, uint8_t B)
{
	uint16_t color;
	color = R;
	color <<= 5;
	color = color + G;
	color <<= 6;
	color = color + B;
	return color;
}
