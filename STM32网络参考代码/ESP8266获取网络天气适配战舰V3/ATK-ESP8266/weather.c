#include "weather.h"
#include "wifista.h"
#include "usart.h"
#include "malloc.h"
#include "usart3.h"
#include "delay.h"
#include "text.h"
#include "parsejson.h"

//�������Ӷ˿ں�:80	
#define WEATHER_PORTNUM 	"80"
//����������IP
#define WEATHER_SERVERIP 	"api.seniverse.com"

//ʱ��˿ں�
#define TIME_PORTNUM			"80"
//ʱ�������IP
#define TIME_SERVERIP			"www.beijing-time.org"


//��ȡһ��ʵʱ����
//���أ�0---��ȡ�ɹ���1---��ȡʧ��
u8 get_current_weather(void)
{
	u8 *p;
	u8 res;
//	u8 ipbuf[16]; 	//IP����
	p=mymalloc(SRAMIN,40);							//����40�ֽ��ڴ�
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",WEATHER_SERVERIP,WEATHER_PORTNUM);    //����Ŀ��TCP������
	res = atk_8266_send_cmd(p,"OK",200);//���ӵ�Ŀ��TCP������
	if(res==1)
	{
		myfree(SRAMIN,p);
		return 1;
	}
	delay_ms(300);
	atk_8266_send_cmd("AT+CIPMODE=1","OK",100);      //����ģʽΪ��͸��	
//	atk_8266_get_wanip(ipbuf);//��ȡWAN IP

	USART3_RX_STA=0;
	atk_8266_send_cmd("AT+CIPSEND","OK",100);         //��ʼ͸��
	printf("start trans...\r\n");
	u3_printf("GET https://api.seniverse.com/v3/weather/now.json?key=pqe1fgv45lrdruq7&location=shenzhen&language=zh-Hans&unit=c\n\n");	
	delay_ms(20);//��ʱ20ms���ص���ָ��ͳɹ���״̬
//	atk_8266_at_response(1);
	USART3_RX_STA=0;	//���㴮��3����
	delay_ms(1000);
//	atk_8266_at_response(0);
	if(USART3_RX_STA&0X8000)		//��ʱ�ٴνӵ�һ�����ݣ�Ϊ����������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
	} 
	parse_now_weather();

	
	atk_8266_quit_trans();//�˳�͸��
	atk_8266_send_cmd("AT+CIPCLOSE","OK",50);         //�ر�����
	myfree(SRAMIN,p);
	return 0;
}

//��ȡ3�������
u8 get_3days_weather(void)
{
	u8 *p;
	u8 res;
	u8 ipbuf[16]; 	//IP����
	p=mymalloc(SRAMIN,40);							//����40�ֽ��ڴ�
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",WEATHER_SERVERIP,WEATHER_PORTNUM);    //����Ŀ��TCP������
	res = atk_8266_send_cmd(p,"OK",200);//���ӵ�Ŀ��TCP������
	if(res==1)
	{
		myfree(SRAMIN,p);
		return 1;
	}
	delay_ms(300);
	atk_8266_send_cmd("AT+CIPMODE=1","OK",100);      //����ģʽΪ��͸��	
	atk_8266_get_wanip(ipbuf);//��ȡWAN IP
	sprintf((char*)p,"IP��ַ:%s �˿�:%s",ipbuf,(u8*)WEATHER_PORTNUM);
//	Show_Str(30,65,200,12,p,12,0);				//��ʾIP��ַ�Ͷ˿�	
	USART3_RX_STA=0;
	atk_8266_send_cmd("AT+CIPSEND","OK",100);         //��ʼ͸��
	printf("start trans...\r\n");

	u3_printf("GET https://api.seniverse.com/v3/weather/daily.json?key=pqe1fgv45lrdruq7&location=shenzhen&language=zh-Hans&unit=c&start=0&days=5\n\n");
	delay_ms(20);//��ʱ20ms���ص���ָ��ͳɹ���״̬
//	atk_8266_at_response(1);
	USART3_RX_STA=0;	//���㴮��3����
	delay_ms(1000);
//	atk_8266_at_response(0);
	if(USART3_RX_STA&0X8000)		//��ʱ�ٴνӵ�һ�����ݣ�Ϊ����������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
	} 
	
	parse_3days_weather();

	
	atk_8266_quit_trans();//�˳�͸��
	atk_8266_send_cmd("AT+CIPCLOSE","OK",50);         //�ر�����
	myfree(SRAMIN,p);
	return 0;
}

//��ȡ����ʱ��
u8 get_beijing_time(void)
{
	u8 *p;
	u8 res;
	u8 ipbuf[16]; 	//IP����
	p=mymalloc(SRAMIN,40);							//����40�ֽ��ڴ�
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",TIME_SERVERIP,TIME_PORTNUM);    //����Ŀ��TCP������
	res = atk_8266_send_cmd(p,"OK",200);//���ӵ�Ŀ��TCP������
	if(res==1)
	{
		myfree(SRAMIN,p);
		return 1;
	}
	delay_ms(300);
	atk_8266_send_cmd("AT+CIPMODE=1","OK",100);      //����ģʽΪ��͸��	
	atk_8266_get_wanip(ipbuf);//��ȡWAN IP
	sprintf((char*)p,"IP��ַ:%s �˿�:%s",ipbuf,(u8*)TIME_PORTNUM);
	Show_Str(30,65,200,12,p,12,0);				//��ʾIP��ַ�Ͷ˿�	
	USART3_RX_STA=0;
	atk_8266_send_cmd("AT+CIPSEND","OK",100);         //��ʼ͸��
	printf("start trans...\r\n");

	u3_printf("GET /time15.asp HTTP/1.1\r\nHost:www.beijing-time.org\n\n");
	delay_ms(20);
//	atk_8266_at_response(1);
	USART3_RX_STA=0;	
	delay_ms(1000);
	atk_8266_at_response(1);
	printf("\r\n\r\n");
	
	atk_8266_quit_trans();//�˳�͸��
	atk_8266_send_cmd("AT+CIPCLOSE","OK",50);         //�ر�����
	myfree(SRAMIN,p);
	return 0;
}
