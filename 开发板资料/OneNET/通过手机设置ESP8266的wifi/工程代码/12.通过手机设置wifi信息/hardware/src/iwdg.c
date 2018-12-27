#include "stm32f10x.h"

#include "iwdg.h"






//�̶�40KHzʱ��
void Iwdg_Init(unsigned char psc, unsigned short arr)
{

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	
	IWDG_SetPrescaler(psc); //����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64
	//0		4��Ƶ
	//1		8��Ƶ
	//2		16��Ƶ
	//3		32��Ƶ
	//4		64��Ƶ
	//5		128��Ƶ
	//6		256��Ƶ
	//7		256��Ƶ
	
	IWDG_SetReload(arr); //����IWDG��װ��ֵ		ÿ��ι���������ֵ��ʼ�ݼ�
	
	IWDG_ReloadCounter(); //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	
	IWDG_Enable(); //ʹ��IWDG

}

//ι�������Ź�
void Iwdg_Feed(void)
{
	
 	IWDG_ReloadCounter();
	
}
