#ifndef _BMP180_H_
#define _BMP180_H_

//BMP180У��
struct BMP180_CAL_PARAM
{
	short int ac1;
	short int ac2;
	short int ac3;
	unsigned int ac4;
	unsigned int ac5;
	unsigned int ac6;
	short int b1;
	short int b2;
	short int mb;
	short int mc;
	short int md;
};

struct BMP180_INFO
{
	unsigned char exist_flag;          //���ڱ�־
	unsigned char version;             //�汾
	struct BMP180_CAL_PARAM cal_param; //����ϵ��
	int unset_temperature;     //δУ�����¶�
	int unset_gas_press;       //δУ������ѹ
	int temperature;           //У������¶�
	int gas_press;             //У�������ѹ
	float altitude;                    //����
};
extern struct BMP180_INFO g_bmp180_data;

#define BMP180_OSS 3  //��Χ(0~3)

extern void bmp180_init(void);
extern void bmp180_convert(void);

#endif
