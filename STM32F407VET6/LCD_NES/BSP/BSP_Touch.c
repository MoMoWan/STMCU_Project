#include  "BSP_Touch.h"


extern SPI_HandleTypeDef hspi2;

uint8_t SPI_WriteByte(uint8_t data) 
{ 
 uint8_t Data = 0; 


   HAL_SPI_TransmitReceive(&hspi2,&data,&Data,1,0xFF);

  // Return the shifted data 
  return Data; 
}  


/****************************************************************************
* ��    �ƣ�uint16_t TPReadX(void) 
* ��    �ܣ�������X�����ݶ���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/  
uint16_t TPReadY(void)
{ 
   uint16_t x=0;
   TP_CS_Slecte;	                        //ѡ��XPT2046 
   HAL_Delay(1);					//��ʱ
   SPI_WriteByte(0xD0);				//����X���ȡ��־
   HAL_Delay(1);					//��ʱ
   x=SPI_WriteByte(0x00);			//������ȡ16λ������ 
   x<<=8;
   x+=SPI_WriteByte(0x00);
   HAL_Delay(1);					//��ֹXPT2046
   TP_CS_Unselected; 					    								  
   x = x>>3;						//��λ�����12λ����Ч����0-4095
   return (x);
}
/****************************************************************************
* ��    �ƣ�uint16_t TPReadX(void)
* ��    �ܣ�������Y�����ݶ���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
uint16_t TPReadX(void)
{
   uint16_t y=0;
   TP_CS_Slecte;	                        //ѡ��XPT2046 
   HAL_Delay(1);					//��ʱ
   SPI_WriteByte(0x90);				//����Y���ȡ��־
   HAL_Delay(1);					//��ʱ
   y=SPI_WriteByte(0x00);			//������ȡ16λ������ 
   y<<=8;
   y+=SPI_WriteByte(0x00);
   HAL_Delay(1);					//��ֹXPT2046
   TP_CS_Unselected; 					    								  
   y = y>>3;						//��λ�����12λ����Ч����0-4095
   return (y);
}

int  GUI_TOUCH_X_MeasureX(void) 
{
	uint8_t t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//������
	unsigned short temp=0,X=0;	
 	
	while(/*GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0&&*/count<10)//ѭ������10��
	{	   	  
		databuffer[count]=TPReadX();
		count++; 
	}  
	if(count==10)//һ��Ҫ����10������,������
	{  
	    do//������X��������
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//��������
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		X=(databuffer[3]+databuffer[4]+databuffer[5])/3;	  
	}
	return(X);  
}

int  GUI_TOUCH_X_MeasureY(void) {
  	uint8_t t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//������
	unsigned short temp=0,Y=0;	
 
    while(/*GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0&&*/count<10)	//ѭ������10��
	{	   	  
		databuffer[count]=TPReadY();
		count++;  
	}  
	if(count==10)//һ��Ҫ����10������,������
	{  
	    do//������X��������
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//��������
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		Y=(databuffer[3]+databuffer[4]+databuffer[5])/3;	    
	}
	return(Y); 
}


//Left:3950  Right:200 Up:200  Down:3700
unsigned int TOUCH_X(void)
{
	unsigned int i;
	i=GUI_TOUCH_X_MeasureX();
	if(i<200||i>3950)return 0;
		else return (3900-i)*24/(390-20);
} 
unsigned int TOUCH_Y(void)
{
	unsigned int i;
	i=GUI_TOUCH_X_MeasureY();
	if(i<200||i>3750)return 0;
		else return (i-200)*32/(375-20);
}

////�������ж�������������
//void Touch_Interrupt_Config(void)
//{
//  GPIO_InitTypeDef  GPIO_InitStructure; 
//  NVIC_InitTypeDef NVIC_InitStructure;
//  EXTI_InitTypeDef EXTI_InitStructure;
//
//
//  //++++++++++�������ж�����+++++++++++
//  // Configure GPIO Pin as input floating 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//  // Connect EXTI Line to GPIO Pin
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
//  // Enable the EXTI8 Interrupt //
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  //���������ж�����ΪPD8
//  // Enable the EXTI Line8 Interrupt //
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//
//} 

