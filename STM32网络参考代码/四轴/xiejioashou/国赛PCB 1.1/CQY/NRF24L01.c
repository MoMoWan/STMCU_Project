/*************************************************
2.4G����ͨ��ģ��24L01����

MCU��STM32F103CBT6��Ӳ��SPI���⺯���汾��3.5.0 KEIL

������2013-01-16

*************************************************/

#include "NRF24L01.h"
#include "stm32f10x.h"
#include "delay.h" 
#include "spi.h"
#include "MPU6050.H"
#include "var_global.h"


#define CE1 GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define CE0 GPIO_ResetBits(GPIOB, GPIO_Pin_2)

#define CSN1 GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define CSN0 GPIO_ResetBits(GPIOA, GPIO_Pin_8)

u8 TxDate[32];

u8 TxAddr[]={0x34,0x43,0x10,0x10,0x01};//���͵�ַ

union//���߷���
{
	RF_data   f; 	
  u8        data[32];
} RF;

RC RCun;


void RF_send()
{  
	RF_SendOnec(RF.data);
  while(CheckACK());	//����Ƿ������
} 


void Get_RFdata()
{ 
 
//NRFSetRXMode();	

if(NRFRevDate(RCun.RxData)){dir_time=0;}

}

void Send_RFdata()
{
	RF.f.Acc_data[0]=ACC_AVG.X;
	RF.f.Acc_data[1]=ACC_AVG.Y;
 	RF.f.Acc_data[2]=ACC_AVG.Z;
  RF.f.F_GRY_data[0]=GRY_F.X;
  RF.f.F_GRY_data[1]=GRY_F.Y;
	RF.f.F_GRY_data[2]=GRY_F.Z;
	RF.f.F_Cal_data[0]=Q_ANGLE.Pitch;
  RF.f.F_Cal_data[1]=Q_ANGLE.Rool; 
	RF_send(); 
}

void RF_SendOnec(u8 *TxDate) //����	
{
  CE0; 
	NRFWriteTxDate(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//д������ 
	CE1;
	//delay_us(5);//����10us������
}

/*****************״̬��־*****************************************/
u8 sta;   //״̬��־

u8 RX_DR;
u8 TX_DS;
u8 MAX_RT;

/*****************SPIʱ����******************************************/

/**********************NRF24L01��ʼ������*******************************/
void NRF24L01Int()
{
	
	CE0; //����ģʽ1   
	CSN1;  
  GPIO_ResetBits(GPIOA,GPIO_Pin_5);//	SCLK0;
	//IRQ1	 
}
/*****************SPI���Ĵ���һ�ֽں���*********************************/
u8 NRFReadReg(u8 RegAddr)
{
   u8 BackDate;
   CSN0;//����ʱ��
   NRFSPI(RegAddr);//д�Ĵ�����ַ
   BackDate=NRFSPI(0xFF);//д����Ĵ���ָ��  
   CSN1;
   return(BackDate); //����״̬
}
/*****************SPIд�Ĵ���һ�ֽں���*********************************/
u8 NRFWriteReg(u8 RegAddr,u8 date)
{
   u8 BackDate;
   CSN0;//����ʱ��
   BackDate=NRFSPI(RegAddr);//д���ַ
   NRFSPI(date);//д��ֵ
   CSN1;
   return(BackDate);
}
/*****************SPI��ȡRXFIFO�Ĵ�����ֵ********************************/
u8 NRFReadRxDate(u8 RegAddr,u8 *RxDate,u8 DateLen)
{  //�Ĵ�����ַ//��ȡ���ݴ�ű���//��ȡ���ݳ���//���ڽ���
    u8 BackDate,i;
	CSN0;//����ʱ��
	BackDate=NRFSPI(RegAddr);//д��Ҫ��ȡ�ļĴ�����ַ
	for(i=0;i<DateLen;i++) //��ȡ����
	  {
	     RxDate[i]=NRFSPI(0);
	  } 
   CSN1;
   return(BackDate); 
}
/*****************SPIд��TXFIFO�Ĵ�����ֵ**********************************/
u8 NRFWriteTxDate(u8 RegAddr,u8 *TxDate,u8 DateLen)
{ //�Ĵ�����ַ//д�����ݴ�ű���//��ȡ���ݳ���//���ڷ���
  u8 BackDate,i;
   CSN0;
   BackDate=NRFSPI(RegAddr);//д��Ҫд��Ĵ����ĵ�ַ
   for(i=0;i<DateLen;i++)//д������
     {
	    NRFSPI(*TxDate++);
	 }   
   CSN1;
   return(BackDate);
}
/*****************NRF����Ϊ����ģʽ����������******************************/
void NRFSetTxMode(u8 *TxDate)
{//����ģʽ
	NRF24L01Int();	
	
  CE0; 
  NRFWriteTxDate(W_REGISTER+TX_ADDR,TxAddr,TX_ADDR_WITDH);//д�Ĵ���ָ��+���յ�ַʹ��ָ��+���յ�ַ+��ַ���
	NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);//Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
	NRFWriteTxDate(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//д������ 
	/******�����йؼĴ�������**************/
  	NRFWriteReg(W_REGISTER+EN_AA,0x01);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);   // ʹ�ܽ���ͨ��0
  	NRFWriteReg(W_REGISTER+SETUP_RETR,0x0a);  //�Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�1��
  	NRFWriteReg(W_REGISTER+RF_CH,0x40);         // ѡ����Ƶͨ��0x40
  	NRFWriteReg(W_REGISTER+RF_SETUP,0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	NRFWriteReg(W_REGISTER+CONFIG_24L01,0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�  
	CE1;
	delay_us(30);//����10us������
}
/*****************NRF����Ϊ����ģʽ����������******************************/
//��Ҫ����ģʽ
void NRFSetRXMode()
{
	  CE0;  
  	NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
  	NRFWriteReg(W_REGISTER+EN_AA,0x01);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);           // ʹ�ܽ���ͨ��0
  	NRFWriteReg(W_REGISTER+RF_CH,0x40);                 // ѡ����Ƶͨ��0x40
  	NRFWriteReg(W_REGISTER+RX_PW_P0,8);  // ����ͨ��0ѡ��8byte���ݿ��
  	NRFWriteReg(W_REGISTER+RF_SETUP,0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������*/
  	NRFWriteReg(W_REGISTER+CONFIG_24L01,0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  	CE1;
    delay_us(20);
}
/****************************���Ӧ���ź�******************************/
u8 CheckACK()
{  //���ڷ���
	sta=NRFReadReg(R_REGISTER+STATUS);                    // ����״̬�Ĵ���
	
RX_DR=(sta&0x40);
TX_DS=(sta&0x20);
MAX_RT=(sta&0x10);
   if(TX_DS||MAX_RT) //��������ж�
	{
	   NRFWriteReg(W_REGISTER+STATUS,0xff);  // ���TX_DS��MAX_RT�жϱ�־
	   CSN0;
	   NRFSPI(FLUSH_TX);//�������FIFO �����ؼ�������Ȼ��������벻���ĺ����������Ҽ�ס����  
     CSN1; 
	   return(0);
	}
	else
	   return(1);
}
/******************�ж��Ƿ�����յ����ݣ��ӵ��ʹ�RXȡ��*********************/
//���ڽ���ģʽ
u8 NRFRevDate(u8 *RevDate)
{
   	 u8 RevFlags=0;
	   sta=NRFReadReg(R_REGISTER+STATUS);//�������ݺ��ȡ״̬�Ĵ���
    
     RX_DR=(sta&0x40);
     TX_DS=(sta&0x20);
     MAX_RT=(sta&0x10);
         
     if(RX_DR)				// �ж��Ƿ���յ�����
	 {
	  CE0; 		   //SPIʹ��
		NRFReadRxDate(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);        // ��RXFIFO��ȡ����
    RevFlags=1;	   //��ȡ������ɱ�־
	  }
	
    NRFWriteReg(W_REGISTER+STATUS,0xff); //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�
	
    CSN0;//CSN=0;
	  NRFSPI(FLUSH_RX);//�������FIFO
	  
		CSN1;
    CE1;		
    return(RevFlags);
}

// void RF_send(u8 adch,u16 i)
// {
// u8 send;
// u8 id;
// u8 chid;
// if (adch==0)chid=0x01;
// if (adch==1)chid=0x05;
// if (adch==2)chid=0x09;
// if (adch==3)chid=0x0d;

// id= chid<<4;
// send=i & 0x0f;
// send|=id; 
// TxDate[0]=send; 
//              
// id= (chid+1)<<4;
// send=(i>>4) & 0x0f;
// send|=id; 
// TxDate[1]=send; 
//  
// id= (chid+2)<<4;
// send=(i>>8) & 0x0f;
// send|=id; 
// TxDate[2]=send; 
//    
// RF_SendOnec();
// while(CheckACK());	//����Ƿ������
// delay_us(200);

// } 

