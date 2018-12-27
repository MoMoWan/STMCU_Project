#include "dac.h"

extern u16 digital;
void MyDAC_Init(void)//DAC channel1 Configuration
{
  	unsigned int tmpreg1=0,tmpreg2=0;
 	RCC->APB2ENR|=1<<2;//ʹ��PORTAʱ��
	RCC->APB1ENR|=RCC_APB1Periph_DAC;//ʹ��DACʱ��
 	GPIOA->CRL&=0XFF00FFFF; 
	GPIOA->CRL|=0X00440000;//PA4,5 ��������   	 

  	tmpreg1=DAC->CR;//Get the DAC CR value  
  	tmpreg1&=~(CR_CLEAR_Mask<<DAC_Channel_1);//Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits  
  	tmpreg2=(DAC_Trigger_Software|DAC_WaveGeneration_None|DAC_LFSRUnmask_Bits8_0|DAC_OutputBuffer_Enable); 
  	tmpreg1|=tmpreg2<<DAC_Channel_1;//Calculate CR register value depending on DAC_Channel 
  	DAC->CR=tmpreg1;//Write to DAC CR 
	DAC->CR|=CR_EN_Set<<DAC_Channel_1;//DAC Channel1ʹ��,PA4�Զ����ӵ�DAC
	DAC1_SetData(0x000);

  	tmpreg1=DAC->CR;//Get the DAC CR value  
  	tmpreg1&=~(CR_CLEAR_Mask<<DAC_Channel_2);//Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits  
  	tmpreg2=(DAC_Trigger_Software|DAC_WaveGeneration_None|DAC_LFSRUnmask_Bits8_0|DAC_OutputBuffer_Enable); 
  	tmpreg1|=tmpreg2<<DAC_Channel_2;//Calculate CR register value depending on DAC_Channel 
  	DAC->CR=tmpreg1;//Write to DAC CR 
	DAC->CR|=CR_EN_Set<<DAC_Channel_2;//DAC Channel2ʹ��,PA5�Զ����ӵ�DAC
	DAC2_SetData(0x000);
}

void DAC1_SetData(u16 data)
{
	DAC->DHR12R1=data;//ͨ��1��12λ�Ҷ�������
	DAC->SWTRIGR|=0x01;//�������ת��
}

void DAC2_SetData(u16 data)
{
	DAC->DHR12R2=data;//ͨ��2��12λ�Ҷ�������
	DAC->SWTRIGR|=0x02;//�������ת��
}






