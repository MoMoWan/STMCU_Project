#include "timer.h"
#include "led.h"
#include "dac.h"
//Mini STM32开发板
//通用定时器 驱动代码			 
//正点原子@ALIENTEK
//2010/6/1

//定时器3中断服务程序
//2ms中断1次
extern u8 DACdone;
extern u8 CHanalnum;
extern u8 Bitnum;
extern u8 wav_buf[1024];
extern u16 DApc;
extern u8 volume;
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 temp;
	if(TIM3->SR&0X0001)//溢出中断
	{
		if(CHanalnum==1)//单声道
		{
			if(Bitnum==8)//8位精度
			{
				DAC->DHR12R1=wav_buf[DApc]*10/volume;//通道1的12位右对齐数据
				DAC->DHR12R2=wav_buf[DApc]*10/volume;//通道1的12位右对齐数据
				DAC->SWTRIGR|=0x03;//软件启动两个通道的转换
				DApc++;
			}
			else if(Bitnum==16)//16位精度(先低位后高位)
			{
				temp=(((u8)(wav_buf[DApc+1]-0x80)<<4)|(wav_buf[DApc]>>4))*10/volume;
				DAC->DHR12L1=temp;
				DAC->DHR12L2=temp;
				DAC->SWTRIGR|=0x03;//软件启动两个通道的转换
				DApc+=2;				
			}
		}
		else if(CHanalnum==2)//立体声	   10110010	 10110010
		{
			if(Bitnum==8)//8位精度
			{
				DAC->DHR12R1=wav_buf[DApc]*10/volume;//通道2的12位右对齐数据
				DApc++;
				DAC->DHR12R2=wav_buf[DApc]*10/volume;//通道2的12位右对齐数据
				DApc++;
				DAC->SWTRIGR|=0x03;//软件启动两个通道的转换
			}
			else if(Bitnum==16)//16位精度(先低位后高位)
			{
				DAC->DHR12L1=(((u8)(wav_buf[DApc+1]-0x80)<<4)|(wav_buf[DApc]>>4))*10/volume;//通道1的12位右对齐数据
				DApc+=2;
				DAC->DHR12L2=(((u8)(wav_buf[DApc+1]-0x80)<<4)|(wav_buf[DApc]>>4))*10/volume;//通道1的12位右对齐数据
				DApc+=2;
				DAC->SWTRIGR|=0x03;//软件启动两个通道的转换				
			}
		}		
		if(DApc==512)DACdone=1;
		if(DApc==1024){DApc=0;DACdone=1;}				    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void Timerx_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;//TIM3时钟使能    
 	TIM3->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM3->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->DIER|=1<<6;   //允许触发中断
		  							    
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//抢占1，子优先级3，组2									 
}

//TIM3 PWM部分
//正点原子@ALIENTEK
//2010/6/2	 

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void PWM_Init(u16 arr,u16 psc)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB1ENR|=1<<1;       //TIM3时钟使能    

	GPIOA->CRH&=0XFFFFFFF0;//PA8输出
	GPIOA->CRH|=0X00000004;//浮空输入
	  	
	GPIOA->CRL&=0X0FFFFFFF;//PA7输出
	GPIOA->CRL|=0XB0000000;//复用功能输出 	  
	GPIOA->ODR|=1<<7;//PA7上拉	

	TIM3->ARR=arr;//设定计数器自动重装值 
	TIM3->PSC=psc;//预分频器不分频
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2模式		 
	TIM3->CCMR1|=1<<11; //CH2预装载使能	   

	TIM3->CCER|=1<<4;   //OC2 输出使能	   

	TIM3->CR1=0x8000;   //ARPE使能 
	TIM3->CR1|=0x01;    //使能定时器3 										  
}  	 













