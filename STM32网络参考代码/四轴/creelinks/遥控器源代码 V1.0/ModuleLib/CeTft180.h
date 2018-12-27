/**
  ******************************************************************************
  * @file    CeTft180.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������CeTft180ģ�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TFT180_H__
#define __CE_TFT180_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_TFT180_VERSION__ 1                                         /*!< �������ļ��İ汾��*/
#define __CE_TFT180_NEED_CREELINKS_VERSION__ 1                          /*!< ��ҪCreelinksƽ̨�����Ͱ汾*/
#if (__CE_CREELINKS_VERSION__ < __CE_TFT180_NEED_CREELINKS_VERSION__)   /*!< ���Creelinksƽ̨��İ汾�Ƿ�����Ҫ��*/
#error "�����ļ�CeTft180.h��Ҫ����1.0���ϰ汾��Creelink�⣬���½www.creelinks.com�������°汾��Creelinks�⡣"
#else

//#define CE_TFT180_SHOW_HORIZONTAL         /*!< �����Ҫ������ʾ��������� ע��V1.0�汾�ݽ�֧������*/

#ifdef CE_TFT180_SHOW_HORIZONTAL
#define CE_TFT180_WIDTH     160
#define CE_TFT180_HIGHT     128
#else
#define CE_TFT180_WIDTH     128
#define CE_TFT180_HIGHT     160
#endif

/**
  * @brief  ö�٣�Tft180��������ʾ����ɫ�б�
  */
typedef enum
{
    CE_TFT180_COLOR_RED     = 0xF800,           /*!< ��ɫ*/
    CE_TFT180_COLOR_GREEN   = 0x07E0,           /*!< ��ɫ*/
    CE_TFT180_COLOR_BLUE    = 0x001F,           /*!< ��ɫ*/
    CE_TFT180_COLOR_WHITE   = 0xFFFF,           /*!< ��ɫ*/
    CE_TFT180_COLOR_BLACK   = 0x0000,           /*!< ��ɫ*/
    CE_TFT180_COLOR_YELLOW  = 0xFFE0,           /*!< ��ɫ*/
    CE_TFT180_COLOR_GRAY0   = 0xEF7D,           /*!< �ػ�ɫ*/
    CE_TFT180_COLOR_GRAY1   = 0x8410,           /*!< ��ɫ*/
    CE_TFT180_COLOR_GRAY2   = 0x4208,           /*!< ǳ��ɫ*/
} CE_TFT180_COLOR_LIST;
/**
  * @brief  ö�٣�Tft180������ʾӢ���ַ�����ģ��С
  */
typedef enum
{
    CE_TFT180_EN_SIZE_F6X8,                     /*!< �ַ���ռ6�����ص㣬��ռ8�����ص�*/
    CE_TFT180_EN_SIZE_F8X16,                    /*!< �ַ���ռ8�����ص㣬��ռ16�����ص�*/
} CE_TFT180_EN_SIZE;

/*
 *CeTft180���Զ���
 */
typedef struct
{
    CeSpiMaster ceSpi;                          /*!< ģ��ʹ�õ���SpiMaster����*/
    CeGpio      ceGpio0;
    CeGpio      ceGpio1;
    CeGpio      ceGpio2;
    char        asBuf[CE_TFT180_WIDTH / 6][CE_TFT180_HIGHT / 8];/*!< ���ڴ�ӡ������Ϣ�Ļ���*/
    uint16      asXIndex;                       /*!< ���ڴ�ӡ������Ϣ�Ĺ��x��*/
    uint16      asYIndex;                       /*!< ���ڴ�ӡ������Ϣ�Ĺ��y��*/
    uint8       isShow;
} CeTft180;
/*
 *CeTft180��������
 */
typedef struct
{
    CE_STATUS   (*initial)(CeTft180* ceTft180, CE_RESOURCE ceSpi, CE_RESOURCE ceGpio0, CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2);/*!<
                                                     @brief CeTft180ģ���ʼ��
                                                     @param ceTft180:CeTft180���Զ���ָ��
                                                     @param ceSpi:CeTft180ģ��ʹ�õ�Spi��Դ��
                                                     @param ceTg:CeTft180ģ��ʹ�õ�Tg��Դ��*/

    void        (*setOn)(CeTft180* ceTft180);   /*!< @brief  ����ʾ
                                                     @param  ceTft180:CeTft180���Զ���*/


    void        (*fill)(CeTft180* ceTft180, uint16 color);/*!<
                                                     @brief  CeTft180��ָ������(��ɫ)����ȫ�����
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  color:ȫ����������(��ɫ)*/

    void        (*drawPoint)(CeTft180* ceTft180, int16 x, int16 y, uint16 color);/*!<
                                                     @brief  CeTft180��ָ��λ�û���
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  x:���ÿ�ʼ��ʾ�ַ�����x������
                                                     @param  y:���ÿ�ʼ��ʾ�ַ�����y������
                                                     @param  color:Ҫ��ʾ������(��ɫ)*/

    void        (*drawRectangle)(CeTft180* ceTft180, uint16 startX, uint16 startY, uint16 endX, uint16 endY, int16 color);/*!<
                                                     @brief  CeTft180���ƾ���
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  startX:�������Ͻ�x����
                                                     @param  startY:�������Ͻ�y����
                                                     @param  endX:�������½�x����
                                                     @param  endY:�������½�y����
                                                     @param  color:Ҫ��ʾ������(��ɫ)*/

    void        (*drawData)(CeTft180* ceTft180, uint16 x, uint16 y, const uint8* colorBuf, uint16 bufSizeWidth, uint16 bufSizeHight);/*!<
                                                     @brief  CeTft180����ͼƬ
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  x:ͼƬ����ʼ��x����
                                                     @param  y:ͼƬ����ʼ��y����
                                                     @param colorBuf:ͼƬ����
                                                     @param bufSizeWidth:ͼƬ��
                                                     @param bufSizeHight:ͼƬ��*/

    void        (*showInt)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const int32 val, CE_TFT180_EN_SIZE showSize);/*!<
                                                     @brief  CeTft180��ʾ32λ�з��ŵ����֣����ֵ0x7FFFFFFF����Сֵ0x80000001
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  x:���ÿ�ʼ��ʾ32λ�з��ŵ����ֵ�x������
                                                     @param  y:���ÿ�ʼ��ʾ32λ�з��ŵ����ֵ�y������
                                                     @param  foreColor:��ʾ�����ǰ��ɫ
                                                     @param  backColor:��ʾ����ı���ɫ
                                                     @param  val:Ҫ��ʾ��32λ�з��ŵ�����
                                                     @param  showSize:��ʾ�������С����ѡCE_LCD_EN_SIZE_F6X8��CE_LCD_EN_SIZE_F8X16*/


    void        (*showString)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const char* msg, CE_TFT180_EN_SIZE showSize);/*!<
                                                     @brief  CeTft180��ʾ�ַ�������֧������
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  x:���ÿ�ʼ��ʾ�ַ�����x������
                                                     @param  y:���ÿ�ʼ��ʾ�ַ�����y������
                                                     @param  foreColor:��ʾ�����ǰ��ɫ
                                                     @param  backColor:��ʾ����ı���ɫ
                                                     @param  msg:Ҫ��ʾ���ַ���ָ��
                                                     @param  showSize:��ʾ�������С����ѡCE_LCD_EN_SIZE_F6X8��CE_LCD_EN_SIZE_F8X16*/

    void        (*showCN1616)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const uint8* cn1616);/*!<
                                                     @brief  CeTft180��ʾ16x16���������壬��ģcn1616�ɴ�CeCN1616.h�л�ȡ
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  x:���ÿ�ʼ��ʾ�ַ�����x������
                                                     @param  y:���ÿ�ʼ��ʾ�ַ�����y������
                                                     @param  foreColor:��ʾ�����ǰ��ɫ
                                                     @param  backColor:��ʾ����ı���ɫ
                                                     @param  cn1616:Ҫ��ʾ������ģ����ģcn1616�ɴ�CeCN1616.h�л�ȡ*/

    void        (*appendString)(CeTft180* ceTft180, const char* msg);/*!<
                                                     @brief  һ�����ڴ�����ԣ��Կ���̨�ķ�ʽ��ӡ������Ϣ
                                                     @param  ceTft180:CeTft180���Զ���
                                                     @param  msg:Ҫ��ʾ����Ϣ*/

    void        (*setOff)(CeTft180* ceTft180);  /*!< @brief  �ر���ʾ
                                                     @param  ceTft180:CeTft180���Զ���*/

    uint8       (*getShowStatus)(CeTft180* ceTft180);
} CeTft180OpBase;
/*
 *CeTft180��������ʵ��
 */
extern const CeTft180OpBase ceTft180Op;

#endif //(__CE_CREELINKS_VERSION__ < __CE_TFT180_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TFT180_H__

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ����) 
* @function ��������ʾR��G��B��ɫ������ʾ������Ӣ��
******************************************************************************
#include "Creelinks.h"
#include "CeTft180.h"

const unsigned char CN1616_B4B4[] = //��
{ 0x0C, 0x06, 0x0C, 0x06, 0x1E, 0x06, 0x1B, 0x36, 0x31, 0xB6, 0x60, 0xF6, 0xFF, 0x36, 0x33, 0x36, 0x33, 0x36, 0x33, 0x36, 0x3F, 0x36, 0x36, 0x36, 0x30, 0xC6, 0x30, 0xC6, 0x1F, 0xDE, 0x00, 0x0C };
const unsigned char CN1616_A3E5[] = //��
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x06, 0x60, 0x0C, 0x30, 0x0F, 0xF0, 0x0C, 0x00, 0x0C, 0x00, 0x06, 0x30, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char CN1616_C1AA[] = //��
{0x00, 0xCC, 0xFE, 0x6C, 0x6C, 0x78, 0x6C, 0x00, 0x7D, 0xFE, 0x6C, 0x30, 0x6C, 0x30, 0x7C, 0x30, 0x6F, 0xFF, 0x6C, 0x30, 0x6E, 0x78, 0x7C, 0x78, 0xEC, 0xCC, 0x0C, 0xCC, 0x0D, 0x86, 0x0F, 0x03};

const unsigned char CN1616_B1B1[] = //��
{0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x66, 0x06, 0x6C, 0x7E, 0x78, 0x06, 0x70, 0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x63, 0x1E, 0x63, 0xF6, 0x63, 0x66, 0x3F, 0x06, 0x00};
const unsigned char CN1616_BEA9[] = //��
{0x03, 0x00, 0x01, 0x80, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1F, 0xF8, 0x01, 0x80, 0x19, 0x98, 0x19, 0x8C, 0x31, 0x86, 0x67, 0x86, 0x03, 0x00};
const unsigned char CN1616_B4F3[] = //��
{0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0xFF, 0xFF, 0x01, 0x80, 0x01, 0x80, 0x03, 0xC0, 0x03, 0xC0, 0x06, 0x60, 0x06, 0x60, 0x0C, 0x30, 0x18, 0x18, 0x30, 0x0C, 0xE0, 0x07};
const unsigned char CN1616_D0C5[] = //��
{0x0C, 0x60, 0x0C, 0x30, 0x0F, 0xFF, 0x18, 0x00, 0x18, 0x00, 0x39, 0xFE, 0x38, 0x00, 0x78, 0x00, 0xD9, 0xFE, 0x18, 0x00, 0x18, 0x00, 0x19, 0xFE, 0x19, 0x86, 0x19, 0x86, 0x19, 0xFE, 0x19, 0x86};
const unsigned char CN1616_BFC6[] = //��
{0x0C, 0x18, 0x1F, 0x98, 0xF8, 0xD8, 0x18, 0xD8, 0x18, 0x18, 0xFF, 0x98, 0x18, 0xD8, 0x3C, 0xD8, 0x3E, 0x18, 0x78, 0x1F, 0x7B, 0xF8, 0xD8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
const unsigned char CN1616_BCBC[] = //��
{0x18, 0x30, 0x18, 0x30, 0x18, 0x30, 0x1B, 0xFF, 0xFE, 0x30, 0x18, 0x30, 0x18, 0x30, 0x1F, 0xFE, 0x1C, 0xC6, 0x38, 0xCC, 0xF8, 0x6C, 0x18, 0x78, 0x18, 0x30, 0x18, 0x78, 0x79, 0xCC, 0x37, 0x07};

CeTft180 myTft180;
int main(void)
{
    ceSystemOp.initial();                                   //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                              //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���
    ceTft180Op.initial(&myTft180, R12Spi, R2TI2c);
    ceTft180Op.setOn(&myTft180);
    while (1)
    {
        ceTaskOp.mainTask();                                //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û���
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_RED);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_GREEN);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_BLUE);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_BLACK);

        ceTft180Op.showCN1616(&myTft180, 40, 40,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_B4B4);
        ceTft180Op.showCN1616(&myTft180, 40 + 16, 40,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_A3E5);
        ceTft180Op.showCN1616(&myTft180, 40 + 32, 40, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_C1AA);

        ceTft180Op.showString(&myTft180, 28, 56, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,"CREELINKS", CE_TFT180_EN_SIZE_F8X16);

        ceTft180Op.showCN1616(&myTft180, 16*1, 72, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_B1B1);
        ceTft180Op.showCN1616(&myTft180, 16*2, 72, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_BEA9);
        ceTft180Op.showCN1616(&myTft180, 16*3, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_B4F3);
        ceTft180Op.showCN1616(&myTft180, 16*4, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_D0C5);
        ceTft180Op.showCN1616(&myTft180, 16*5, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_BFC6);
        ceTft180Op.showCN1616(&myTft180, 16*6, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_BCBC);

        ceSystemOp.delayMs(2000);
    };
}
******************************************************************************
*/
