#include "stm32f10x.h"

/************************************************************************************

* ժ Ҫ:��չ 1K FLASH ��Ϊ EEPROMG  

*	 StartAddr--��FLASH������ݵ��׵�ַ������Ϊ���飬ÿ��2K��һ���׵�ַ
	 EndAddr--��FLASH������ݵ�ĩ��ַ������Ϊ���飬�����趨Ϊ��ǰҳ֮�ڵ��κ�һ����ַ��������Ҫ��ų���Ĵ�С���趨
	 *Data--Ҫд��FLASH�����ݣ�ָ������������ڴ�ĵ�ַ
			 

���� 2013-05-15
*************************************************************************************/
#define FLASH_PAGE_SIZE    ((u16)0x400) //1024  1KΪһҳ ��0-3FF��
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;



u32 StartAddr =0x08008000;  //Ҫд��Flash�����ݵ��׵�ַ
u32 EndAddr   =0x080083FF;//Ҫд��Flash�����ݵ�ĩ��ַ

u32 FlashAddress=0x00;//Flash���ڲ���ַ	 
//u32 Data=0x12345678;	 //Ҫд������ݣ�ָ������������ڴ�ĵ�ַ


 
union 
{
u32 Data[8];
float F_Data[8];
}ftoc;



vu32 NbrOfPage = 0x00; //Ҫ������ҳ������

//u32 *p=(u32 *)0x08008000; //����ָ��ָ��Ҫ���͵����ݵĵ�ַ

volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;

ErrorStatus HSEStartUpStatus;





/*******************************************************************************
* Function Name  : Writeflash
* Description    : д���� ������д��FLASH��
*                  �Ȳ���1KȻ��д��

*******************************************************************************/
void Writeflash()
{ u8 num,t_num=0; 
	
	num=sizeof(ftoc.Data)/sizeof(ftoc.Data[0]);//����������м���Ԫ��
	
	FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED; 

    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();	

    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//���־λ
	
//������д����		
//*******************************************************************************  

      /* Define the number of page to be erased *///Ҫ����ҳ���������1
      //NbrOfPage = (EndAddr - StartAddr) / FLASH_PAGE_SIZE;
	   
       /* Erase the FLASH pages *///ҳ�����,��0x8008000-0x8008F33
    	FLASHStatus = FLASH_ErasePage(StartAddr); 
	    
	    //д����	
      FlashAddress = StartAddr;
		
			
	  
      while((FlashAddress < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
      {
     	  if(t_num<num) FLASHStatus = FLASH_ProgramWord(FlashAddress, ftoc.Data[t_num]);
        else          FLASHStatus = FLASH_ProgramWord(FlashAddress, 0x1234);
        
				t_num++;
				FlashAddress = FlashAddress + 4;
      }

	 }


/*******************************************************************************
* Function Name  : Readflash
* Description    : �����ݣ���FLASH�ж�����Ҫ������
*                
* Input          : None
* Output         : Data���Ҫȡ��������
* Return         : None
*******************************************************************************/
/*
void Readflash(void)
{

    FlashAddress = StartAddr;
	//������

    Data=*p;
//    printf ("\r\n Bellow is the data from memory*************************************\n\r\n");

	while(p<( u32 *)EndAddr)//&&(p>(u32 *)StartAddr))
	{
//	    printf("%x",Data);

	    p++;
	}
	 
 
}
*/
