#ifndef _TFT_h
#define _TFT_h

#include "headfile.h"

/******������ɫ*****/
#define RED     0XF800    //��ɫ
#define GREEN   0X07E0    //��ɫ
#define BLUE    0X001F    //��ɫ
#define BRED    0XF81F
#define GRED    0XFFE0    //��ɫ
#define GBLUE   0X07FF    //
#define BLACK   0X0000    //��ɫ
#define WHITE   0XFFFF    //��ɫ
#define YELLOW  0xFFE0    //��ɫ


//����д�ֱʵ���ɫ
#define PENCOLOR RED

//���屳����ɫ
#define BGCOLOR	 WHITE

extern const unsigned char gImage_qq[];

void lcd_initial(void);
void dsp_single_colour(int color);
void showimage(const unsigned char *p); //��ʾ40*40 QQͼƬ
void lcd_showchar(word x,word y,byte dat);
void lcd_showint8(word x,word y,char dat);
void lcd_showint16(word x,word y,int dat);
void lcd_showstr(word x,word y,byte dat[]);           

#endif