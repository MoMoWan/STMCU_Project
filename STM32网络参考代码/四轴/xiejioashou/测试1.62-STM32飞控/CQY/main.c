#include "stm32f10x.h"
#include "stdlib.h"
#include "math.h"
#include "delay.h" 
#include "NRF24L01.H"
#include "MPU6050.H"
#include "IIC.H"
#include "spi.H"
#include "ALL_config.h"
#include "var_global.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "HMC5883.H"

//-----------------------------------------------------------------//
u16 dir_time=0;
//---------------------------------------------------------------------//

void IIC_Reboot() //����iic����
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
 
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
  GPIO_ResetBits(GPIOB,GPIO_Pin_7);

  GPIO_SetBits(GPIOB,GPIO_Pin_6);
  GPIO_SetBits(GPIOB,GPIO_Pin_7);	


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);	

}



int main()


{

RCC_Configuration();

I2C_Configuration();

GPIO_Configuration();	

SPIx_Init();	
NRF24L01Int();		
NRFSetRXMode();	

Tim2_init();
IIC_Reboot();
 
Init_MPU6050() ;	
	
Get_Offset();

//NRFSetTxMode(TxDate);		

TIM3_INT();	
TIM4_INT();	
PID_INIT();	



while(1)
{
	
	
if(FLY_Enable )	
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	Delay_s(10);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);	
  Delay_s(10);
}
		

	else GPIO_SetBits(GPIOB,GPIO_Pin_0);		
	

}

}
 

void TIM3_IRQHandler()//1ms
{TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 
	
	READ_MPU6050();	
	
	
	Cal_TsData();	
	
	IMUupdate(GRY_F.X,GRY_F.Y,GRY_F.Z,ACC_AVG.X*5,ACC_AVG.Y*5,ACC_AVG.Z*5);

	PID_CAL();
 
	Get_RFdata();
		
	
	dir_time++;
  
	if( (dir_time%10)==0 )read_hmc5883l();
	
  if(dir_time>500) {dir_time=0; PWM_X=0;PWM_Y=0; }
}

//RF_send(1,(Q_ANGLE.Pitch)*20+2048);
//RF_send(2,((ACC_RealData.X)/(-7))+2048);
//RF_send(3,((ACC_RealData.Y)/8)+2048);
//RF_send(0,(Q_ANGLE.Rool)*12+2048);	
//RF_send(2,(Q_ANGLE.Yaw )*12+2048);


//RF_send(3,Data_time/4);
