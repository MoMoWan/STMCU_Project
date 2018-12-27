#include "adc.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
 

int16_t  accelerator;
int16_t  Pitch_ta;
int16_t  Roll_ta;
int16_t  Yaw_ta;

int16_t  accelerator_a;
int16_t  Pitch_ta_a;
int16_t  Roll_ta_a;

u16 ADC_ConvertedValue[4];

static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);		
}

static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* DMA channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize         = 4;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
     
  /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;        //����ģʽ ÿ��ADC��������
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;				              //ʹ��ɨ��ģʽ  scanλ����
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	                    // contλ���� ����ת��ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;	; //EXTSEL ѡ����������ͨ����ת�����ⲿ�¼� ���ó����������
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;	        //���ݶ��� �������λ�����   �������ó��Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel       = 4;		                        //����ͨ�����г��� ��Щλ����������ڹ���ͨ��ת�������е�ͨ����Ŀ 1��ת�� ָ���ɶ��ٸ�ͨ����ת��
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);	//ת��ʱ����55.5������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_55Cycles5);	//ת��ʱ����55.5������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_55Cycles5);	//ת��ʱ����55.5������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_55Cycles5);	//ת��ʱ����55.5������
	
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE); 
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}
//**************************************
//���ƾ�ֵ�˲�
//**************************************
#define N 10
float Data_accelerator[N];
float Data_Pitch_ta[N];
float Data_Roll_ta[N];
float GildeAverageValueFilter(float NewValue,float *Data)
{
	unsigned char i;
	float Value;
	float sum;
	sum=0;
	Data[N] = NewValue;
	for(i=0;i<N;i++)
	{
	Data[i]=Data[i+1];
	sum+=Data[i];
	}
	Value=sum/N;
	return(Value);
}
void ADC1_Value(void)
{
	 accelerator_a = ADC_ConvertedValue[2];
	 accelerator   = GildeAverageValueFilter(accelerator_a,Data_accelerator);
	 Pitch_ta_a    = ADC_ConvertedValue[1];
	 Pitch_ta      = GildeAverageValueFilter(Pitch_ta_a,Data_Pitch_ta)-1500;
	 Roll_ta_a     = ADC_ConvertedValue[0];
	 Roll_ta       = GildeAverageValueFilter(Roll_ta_a,Data_Roll_ta)-2100;
	 Yaw_ta        = 0;//ADC_ConvertedValue[3];	
}


