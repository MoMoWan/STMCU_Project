#include <stm32f10x_lib.h>
#include "fsmc.h"

void LCD_FSMCCfg(void)
{
    //Bank1 NOR/SRAM ���ƼĴ�������
	FSMC_Bank1->BTCR[6]=0x00000000|0x00000000|0x00000010|0x00000000|0x00000000|
						0x00000000|0x00000000|0x00001000|0x00000000|0x00000000|0x00000000;                        
	//Bank1 NOR/SRAM ʱ��Ĵ�������  
	FSMC_Bank1->BTCR[7]=0x00000000|0x00000000|(0x00000002<<8)|
						0x00000000|0x00000000|0x00000000|0x00000000;
	//Bank1 NOR/SRAMдʱ��Ĵ�������(��ʹ����չģʽʱ,�˴�û��)
	FSMC_Bank1E->BWTR[6]=0x0FFFFFFF;
  	//ʹ��BANK4(NOR/SRAM Bank 1~4�е�)
	FSMC_Bank1->BTCR[6]|=0x00000001;
}
