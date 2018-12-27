#ifndef  __BSP_TOUCH_H__
#define  __BSP_TOUCH_H__

#include "main.h"
#include "stm32f4xx_hal.h"

//#include "GUI.h"

// A/D ͨ��ѡ�������ֺ͹����Ĵ���
// #define	CHX 	0xd0 	//ͨ��Y+��ѡ�������	
// #define	CHY 	0x90	//ͨ��X+��ѡ�������
#define	CHX 	0x90 	//ͨ��Y+��ѡ�������	
#define	CHY 	0xd0	//ͨ��X+��ѡ�������


#define TP_CS_Slecte       HAL_GPIO_WritePin(Touch_CS_GPIO_Port, Touch_CS_Pin, GPIO_PIN_RESET)
#define TP_CS_Unselected   HAL_GPIO_WritePin(Touch_CS_GPIO_Port, Touch_CS_Pin, GPIO_PIN_SET)

unsigned int TOUCH_X(void); 
unsigned int TOUCH_Y(void); 
/*
int  GUI_TOUCH_X_MeasureX(void) ;
int  GUI_TOUCH_X_MeasureY(void) ;
*/
void TP_GetAdXY(unsigned int *x,unsigned int *y);

char IsPressed(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);
char IsPressed_V2(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);
char IsPressed_V3(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);


#endif                                     
