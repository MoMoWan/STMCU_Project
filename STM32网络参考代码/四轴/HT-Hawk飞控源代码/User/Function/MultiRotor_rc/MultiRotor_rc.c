/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��RC.c
 * ����    ������ң��������         
 * ʵ��ƽ̨��HT_Hawk
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * ��̳    ��http://www.airnano.cn
 * �Ա�    ��http://byd2.taobao.com  
 *           http://hengtuo.taobao.com  
**********************************************************************************/
#include "MultiRotor_rc.h"
#include "include.h"

RC_GETDATA RC_Data;      //ң�������ݽṹ�嶨��

////***********************************************************************//
////typedef struct {                     //���ڴ��������ң������
////	            int16_t rc_data[4];  //���벶����ͨ�����ݴ��
////				int16_t ROLL;        //���벶�����ݴ������
////				int16_t PITCH;
////				int16_t THROTTLE;
////				int16_t YAW;
////				int16_t SENSITIVITY; //��SENSITIVITY�����жȲ���
////                }RC_GETDATA;   
////***********************************************************************//

////1��typedef int NUM[10];//����������������

////    NUM n;//����nΪ�����������������n[0]--n[9]����

////2��typedef char* STRING;//����STRINGΪ�ַ�ָ������

////    STRING p,s[10];//pΪ�ַ�ָ�������sΪָ������

////3��typedef int (*POINTER)();//����POINTERΪָ������ָ�����ͣ��ú�����������ֵ,û�в���

////    POINTER P1,P2;//p1��p2ΪPOINTER���͵�ָ�����

// typedef void (*rcReadRawData)(void);
//ע��˴�����ָ����÷�
rcReadRawData rcReadRawFunc = RC_Data_Refine;


void RDAU(void)             //����ȡң�������ݡ�
{
	RC_directive();           //������������
	rcReadRawFunc();          //rcReadRawFunc = RC_Data_Refine
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : RC_directive
**���� : ��ң��ָ�
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void RC_directive(void)
{
    u8 stTmp = 0,i;                    //��stTmp  i��      ��������
	static u8  rcSticks;               //��rcSticks��      ��̬�ֲ�����
	static u8  rcDelayCommand;         //��rcDelayCommand ��
    static u16 seltLockCommend;	       //��seltLockCommend��
	
	for (i = 0; i < 4; i++) {          //ѭ�������ͨ��ң������
			stTmp >>= 2;
			if (RC_Data.rc_data[i] > RC_MINCHECK)     //RC_MINCHECK   1200
					stTmp |= 0x80;  // check for MIN  //��λ�ڰ�λ     1XXX XXXX
            //��ͨ����ֵ��ű궨
			if (RC_Data.rc_data[i] < RC_MAXCHECK)     //RC_MAXCHECK   1800
					stTmp |= 0x40;  // check for MAX  //��λ����λ     XXXX 1XXX
	}
    
	if (stTmp == rcSticks)  
    {
		    if (rcDelayCommand < 250)
					rcDelayCommand++;
	} 
    else
			rcDelayCommand = 0;
	        rcSticks = stTmp;
	
	if (rcDelayCommand == 150) 
     {
		if (flag.ARMED)    //���������־
            {
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //��������
				  flag.ARMED=0;
		    }
            
		else
            {
            if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)    //��������   
					flag.ARMED=1;
            
			if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)    //�����ٶȽ�����  
					flag.calibratingA = 1;
            
			if ((rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_HI) && flag.calibratingM_pre)  //���شżƽ�����
				  flag.calibratingM = 1;
            
			if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_LO)    
					flag.calibratingM_pre = 1;
			else flag.calibratingM_pre = 0;
			
            }
	}
	//��װ֮��һ��ʱ�����ű������  ���Զ������װ
	if (flag.ARMED){
	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
		    if (seltLockCommend < AUTODISARMDE_TIME)
					 seltLockCommend++;
				else 
					 flag.ARMED=0;
		 }
		 else 
        seltLockCommend = 0;			 
	}
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : RcData_Refine
**���� : ������ң�����ݡ�     �������ݴ�������  ���մ��������  �м侭����ʲô���Ĵ���
**���� : None
**��� : None
**��ע : ��
//��ͨ������ֵ����ƽ��ֵ����
**====================================================================================================*/
/*====================================================================================================*/
void RC_Data_Refine(void)
{
  u8 chan,a;	

	u16 rcDataMax[6], rcDataMin[6];
	static int16_t rcDataCache[6][4], rcDataMean[6];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 6; chan++) {
		  //����ƽ��ֵ�˲���4��
		  if(RC_Pwm_In[chan]>2800 || RC_Pwm_In[chan]<800) 
              RC_Pwm_In[chan] = RC_Pwm_In_his[chan];
			  rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan];		
		      RC_Pwm_In_his[chan] = RC_Pwm_In[chan];
			
			  rcDataMean[chan] = 0;
		      rcDataMax[chan] = 0;
		      rcDataMin[chan] = 25000;
		
			for (a = 0; a < 4; a++) 
            {
				  // ��¼���������ֵ && ��Сֵ
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // ���
					rcDataMean[chan] += rcDataCache[chan][a];  
              }
			// �޳������� ���ֵ && ��Сֵ 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 
//==================================================================================================//
    //���ݴ��������RC_Data�ṹ���rcDataMean��������
	 RC_Data.YAW   = RC_Data.rc_data[2] =rcDataMean[3];             //����ң�����������ݴ�š���
	 RC_Data.THROTTLE  = RC_Data.rc_data[3] =rcDataMean[2];
	 RC_Data.ROLL  = RC_Data.rc_data[0] = rcDataMean[0];            //����ֵ���ڶ��ķ�Χ֮�ڡ�
	 RC_Data.PITCH = RC_Data.rc_data[1] = rcDataMean[1];            //��1000����2000��
}

////***********************************************************************//
////typedef struct {                     //���ڴ��������ң������
////	            int16_t rc_data[4];  //���벶����ͨ�����ݴ��
////				int16_t ROLL;        //���벶�����ݴ������
////				int16_t PITCH;
////				int16_t THROTTLE;
////				int16_t YAW;
////				int16_t SENSITIVITY; //��SENSITIVITY�����жȲ���
////                }RC_GETDATA;   
////***********************************************************************//