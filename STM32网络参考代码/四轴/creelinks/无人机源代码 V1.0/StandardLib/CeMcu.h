/**
  ******************************************************************************
  * @file    CeMcu.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks�봦����Ӳ��ƽ̨��ص���������
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_MCU_H__
#define __CE_MCU_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#include "stm32f10x.h"

/*Creelinksƽ̨������������Դ����****************************************/
#define CE_MCU_CLOCK_FREQUENCY_MHZ  72                          /*!< MCUʱ�����Ƶ�ʣ���λMHz*/

#define CE_MCU_ROM_SIZE_KB          512                         /*!< MCU ROM����*/
#define CE_MCU_RAM_SIZE_KB          192                         /*!< MCU RAM����*/

/*Creelinksƽ̨System��Դ����****************************************/
#define CE_SYSTEM_DELAY_USE_TIMX    Timer5                      /*!< ϵͳdelay����ʹ��һ����ʱ��ʵ�֣��˴���ָ��ʹ���ĸ���ʱ������ѡTIM2~TIM7*/


/*Creelinksƽ̨Adת����Դ����****************************************/
#define CE_AD_CONVERT_REF_VCC       (fp32)(3.3f)                /*!< �궨��Adת���Ĳο���ѹ3.3V*/
#define CE_AD_CONVERT_WIDTH         (uint16)(12)                /*!< �궨��Adת�����*/
#define CE_AD_CONVERT_TIME_NS       (uint16)1000                /*!< �궨��Ad���һ��ת�����õ�ʱ�䣬��λns*/
#define CE_AD_CONVERT_MAX_VAL       (uint32)(0x0FFF)            /*!< �궨��Adת�����õ����ֵ*/

/*Creelinksƽ̨Ccp��Դ���ò���***************************************/
#define CE_CCP_MAX_COUNT            (uint32)65535               /*!< CCP��֧�ֵ�������ֵ*/

/*Creelinksƽ̨Daת������Դ���ò���**********************************/
#define CE_DA_CONVERT_WIDTH         (uint32)12                  /*!< Da��Դ��ת�����*/
#define CE_DA_CONVERT_TIME_NS       (uint32)0                   /*!< Da��Դ������Сת��ʱ��*/
#define CE_DA_MIN_INTERVAL_NS       (uint32)14                  /*!< ��������ת��ʱ����Сֵ*/
#define CE_DA_MAX_INTERVAL_NS       (uint32)59652323555         /*!< ��������ת��ʱ������ֵ*/
#define CE_DA_CONVERT_MAX_VAL       (uint32)0x0FFF              /*!< Da��Դ��ת�������ֵ*/

/*Creelinksƽ̨Flash��Դ���ò���************************************/
#define CE_FLASH_SIZE               (uint32)2048                /*!< �궨��Flash����������λByte*/
#define CE_FLASH_READ_SIZE          (uint32)2                   /*!< �궨�壬����һ��Flash��ȡ����С���ȣ�����ַ��Ϊ��ֵ��������*/
#define CE_FLASH_WRITE_SIZE         (uint32)2                   /*!< �궨�壬����һ��Flashд�����С���ȣ���λByte��д��ַ��Ϊ��ֵ��������*/
#define CE_FLASH_ERASE_SIZE         (uint32)2048                /*!< �궨�壬����һ��Flash��������С���ȣ���λByte������ʱ���׵�ַ������Ҫ�����ĳ�����Ϊ��ֵ��������*/

/*Creelinksƽ̨Gpio��Դ���ò���*************************************/
#define CE_GPIO_SPEED_MHZ           (uint32)50                  /*!< GPIO�ڵ�����ƽ��ת����*/
#define CE_SET_GPIO_BIT(ceGpio)     ((ceGpio)->ceExGpioPar.ceExGpiox->BSRR = (ceGpio)->ceExGpioPar.ceExGpioPinx)   /*!< �궨��Gpio���øߵ�ƽ�����ڶ�GPIOЧ��Ҫ���Ϊ�ϸ�ĳ���*/
#define CE_RESET_GPIO_BIT(ceGpio)   ((ceGpio)->ceExGpioPar.ceExGpiox->BRR = (ceGpio)->ceExGpioPar.ceExGpioPinx)    /*!< �궨��Gpio���õ͵�ƽ�����ڶ�GPIOЧ��Ҫ���Ϊ�ϸ�ĳ���*/
#define CE_GET_GPIO_BIT(ceGpio)     ((ceGpio)->ceExGpioPar.ceExGpiox->IDR  & (ceGpio)->ceExGpioPar.ceExGpioPinx == 0 ? 0x00:0x01)        /*!< �궨���ȡGpio��ƽ�����ڶ�GPIOЧ��Ҫ���Ϊ�ϸ�ĳ���*/
#define CE_SET_GPIO_MODE(ceGpio,ceGpioMode)  ceGpio_setMode((ceGpio), (ceGpioMode))                          /*!< �궨������Gpio�ӿڹ���ģʽ�����ڶ�GPIOЧ��Ҫ���Ϊ�ϸ�ĳ���*/

/*Creelinksƽ̨Pwm��Դ���ò���**************************************/
#define CE_PWM_MAX_CYCLE_NS     (uint32)59652323555         /*!< Pwm��֧�ֵ��������*/
#define CE_PWM_MIN_CYCLE_NS     (uint32)14                  /*!< Pwm��֧�ֵ���С����*/
#define CE_PWM_MIN_DIVIDE_NS    (uint32)14                  /*!< Pwm��֧�ֵ���С����*/

/*Creelinksƽ̨Uart��Դ���ò���*************************************/
#define CE_UART_MAX_BAUD_RATE   (uint32)115200              /*!< UART��֧�ֵ��������*/
#define CE_UART_MIN_BAUD_RATE   (uint32)2400                /*!< UART��֧�ֵ���С������*/

/*Creelinksƽ̨���뿪�ؿ���*****************************************/
#define __CE_CHECK_PAR__                    /*!< �Ƿ���в�����֤�궨�壬�����ʹ�ú����β���֤����ע�����У��Լ��ٱ�������ɵĴ�����*/
#define __CE_USE_DEBUG__                    /*!< �Ƿ�ʹ��UARTx�ڽ��д�����Ժ궨�壬����ʹ�ã���ע�����У��Լ��ٱ�������ɵĴ�����*/

/*Creelinksƽ̨����ϵͳ���*****************************************/
//#define __CE_USE_RTOS__                   /*!< �Ƿ���ʵʱ����ϵͳ�Ļ���*/
#define CE_TICKER_CALL_TIME_MS  5           /*!< Ticker����RTOS���õ�����*/
#ifdef __CE_USE_RTOS__
#include "ucos_ii.h"
#define CE_STK                  OS_STK      /*!< �����ջ����*/
#define CE_TASK_NAME_LENGTH     OS_TASK_NAME_SIZE /*!< ������������ĳ��ȣ�������32*/
#endif //__CE_USE_RTOS__

/*Creelinksƽ̨��Դ������*****************************************/
#define CE_RES_MARK_BASE    0xFFFFFF00      /*!< ��Դ��֤���ֵ*/
#define CE_RES_MARK_GPIO    0x01000100      /*!< GPIO��Դ*/
#define CE_RES_MARK_PWM     0x01000200      /*!< PWM��Դ*/
#define CE_RES_MARK_CCP     0x01000400      /*!< CCP��Դ*/
#define CE_RES_MARK_INT     0x01000800      /*!< INT��Դ*/
#define CE_RES_MARK_AD      0x01001000      /*!< AD��Դ*/
#define CE_RES_MARK_DA      0x01001100      /*!< DA��Դ*/

#define CE_RES_MARK_I2C     0x02000100      /*!< I2C��Դ*/
#define CE_RES_MARK_UART    0x02000200      /*!< UART��Դ*/
#define CE_RES_MARK_SPI     0x02000400      /*!< SPI��Դ*/
#define CE_RES_MARK_CAN     0x02000800      /*!< CAN��Դ*/
#define CE_RES_MARK_I2S     0x02001000      /*!< I2S��Դ*/
#define CE_RES_MARK_TG      0x02002000      /*!< TG��Դ*/
#define CE_RES_MARK_USB     0x02008000      /*!< USB��Դ*/
#define CE_RES_MARK_TIMER   0x04000100      /*!< Mcu�ڲ���ʱ����Դ*/
#define CE_RES_MARK_G8       0x04000200      /*!< Mcu�ڲ���ʱ����Դ*/
#define CE_RES_MARK_G16     0x04000400      /*!< Mcu�ڲ���ʱ����Դ*/
#define CE_RES_MARK_G32     0x04000800      /*!< Mcu�ڲ���ʱ����Դ*/

#define CE_NULL     (void*)0                /*!< ��ָ��궨��*/

typedef unsigned    char        uint8;
typedef signed      char        int8;
typedef unsigned    short       uint16;
typedef signed      short       int16;
typedef unsigned    int         uint32;
typedef signed      int         int32;
typedef unsigned    long long   uint64;
typedef signed      long long   int64;
typedef             float       fp32;
typedef             double      fp64;


/**
 * @brief  �����ļ�״ָ̬ʾ��
 */
typedef enum
{
    CE_STATUS_FAILE   = 0x00,               /*!< ����ʧ��*/
    CE_STATUS_SUCCESS = 0x01,               /*!< �����ɹ�*/
    CE_STATUS_RESOURCE_ERROR,               /*!< ��Դָ������*/
    CE_STATUS_INITIAL_FALSE,                /*!< ��ʼ��ʧ��*/
    CE_STATUS_NULL_POINTER,                 /*!< ָ��Ϊ�մ���*/
    CE_STATUS_MALLOC_FALSE,                 /*!< ���ݷ���ʧ��*/
    CE_STATUS_PAR_ERROR,                    /*!< ��������*/
    CE_STATUS_OUT_TIME,                     /*!< ��ʱ״̬*/
} CE_STATUS;

/**
  * @brief  ö�٣�GPIOģʽ����
  */
typedef enum
{
    CE_GPIO_MODE_AIN         = 0x0,         /*!< ģ������*/
    CE_GPIO_MODE_IN_FLOATING = 0x04,        /*!< ��������*/
    CE_GPIO_MODE_IPD         = 0x28,        /*!< ��������*/
    CE_GPIO_MODE_IPU         = 0x48,        /*!< ��������*/
    CE_GPIO_MODE_OUT_OD      = 0x14,        /*!< ��©���*/
    CE_GPIO_MODE_OUT_PP      = 0x10,        /*!< �������*/
    CE_GPIO_MODE_AF_OD       = 0x1C,        /*!< ���ÿ�©���*/
    CE_GPIO_MODE_AF_PP       = 0x18         /*!< �����������*/
}CE_GPIO_MODE;

/**
  * @brief  �����ļ������İ���õ���Դ�ţ���PCB��˿ӡ��Ӧ
  */
typedef enum
{
        CE_NULL_RESOURCE    = 0x00000000,
        PA0ACGIP            = 0x00000000 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT | CE_RES_MARK_CCP,
        PA1AGIP             = 0x00000001 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA2AGIP             = 0x00000002 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA3AGIP             = 0x00000003 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA4ADGI             = 0x00000004 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA5ADGI             = 0x00000005 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA6AGI              = 0x00000006 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA7AGI              = 0x00000007 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PA8ClkGIP           = 0x00000008 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PA9GIP              = 0x00000009 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PA10GIP             = 0x0000000A | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PA11GIP             = 0x0000000B | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PA12CGI             = 0x0000000C | CE_RES_MARK_GPIO | CE_RES_MARK_INT | CE_RES_MARK_CCP,
        PA13GI              = 0x0000000D | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PA14GI              = 0x0000000E | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PA15GIP             = 0x0000000F | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,

        PB0AGIP             = 0x00000010 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PB1AGIP             = 0x00000011 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PB2GI               = 0x00000012 | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PB3GIP              = 0x00000013 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB4GIP              = 0x00000014 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB5GIP              = 0x00000015 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB6GIP              = 0x00000016 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB7GIP              = 0x00000017 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB8GIP              = 0x00000018 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB9GI               = 0x00000019 | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PB10GIP             = 0x0000001A | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB11GIP             = 0x0000001B | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PB12GI              = 0x0000001C | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PB13GI              = 0x0000001D | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PB14GI              = 0x0000001E | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PB15GI              = 0x0000001F | CE_RES_MARK_GPIO | CE_RES_MARK_INT,

        PC0AGI              = 0x00000020 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC1AGI              = 0x00000021 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC2AGI              = 0x00000022 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC3AGI              = 0x00000023 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC4AGI              = 0x00000024 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC5AGI              = 0x00000025 | CE_RES_MARK_GPIO | CE_RES_MARK_AD | CE_RES_MARK_INT,
        PC6GIP              = 0x00000026 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PC7GIP              = 0x00000027 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PC8GIP              = 0x00000028 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PC9GIP              = 0x00000029 | CE_RES_MARK_GPIO | CE_RES_MARK_PWM | CE_RES_MARK_INT,
        PC10GI              = 0x0000002A | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PC11GI              = 0x0000002B | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PC12GI              = 0x0000002C | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PC13GI              = 0x0000002D | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PC14GI              = 0x0000002E | CE_RES_MARK_GPIO | CE_RES_MARK_INT,
        PC15GI              = 0x0000002F | CE_RES_MARK_GPIO | CE_RES_MARK_INT,

        PD2CGI              = 0x00000032 | CE_RES_MARK_GPIO | CE_RES_MARK_INT | CE_RES_MARK_CCP,

        PAG8H               = 0X00000000 | CE_RES_MARK_G8,
        PAG8L               = 0X00000001 | CE_RES_MARK_G8,
        PBG8H               = 0X00000002 | CE_RES_MARK_G8,
        PBG8L               = 0X00000003 | CE_RES_MARK_G8,
        PCG8H               = 0X00000004 | CE_RES_MARK_G8,
        PCG8L               = 0X00000005 | CE_RES_MARK_G8,

        PAG16               = 0X00000006 | CE_RES_MARK_G16,
        PBG16               = 0X00000007 | CE_RES_MARK_G16,
        PCG16               = 0X00000008 | CE_RES_MARK_G16,

        I2c1                = 0x00000000 | CE_RES_MARK_I2C,
        I2c2                = 0x00000001 | CE_RES_MARK_I2C,
        I2c3                = 0x00000002 | CE_RES_MARK_I2C,

        Spi1                = 0x00000000 | CE_RES_MARK_SPI,
        Spi2                = 0x00000001 | CE_RES_MARK_SPI,
        Spi3                = 0x00000002 | CE_RES_MARK_SPI,

        Timer1              = 0x00000000 | CE_RES_MARK_TIMER,
        Timer2              = 0x00000001 | CE_RES_MARK_TIMER,
        Timer3              = 0x00000002 | CE_RES_MARK_TIMER,
        Timer4              = 0x00000003 | CE_RES_MARK_TIMER,
        Timer5              = 0x00000004 | CE_RES_MARK_TIMER,
        Timer6              = 0x00000005 | CE_RES_MARK_TIMER,
        Timer7              = 0x00000006 | CE_RES_MARK_TIMER,
        Timer8              = 0x00000007 | CE_RES_MARK_TIMER,

        Uart1               = 0x00000000 | CE_RES_MARK_UART,
        Uart2               = 0x00000001 | CE_RES_MARK_UART,
        Uart3               = 0x00000002 | CE_RES_MARK_UART,
        Uart4               = 0x00000003 | CE_RES_MARK_UART,
        Uart5               = 0x00000004 | CE_RES_MARK_UART,
}CE_RESOURCE;

/**
  * @brief  �ṹ�壬Pwm�����ڲ���չ���Լ���
  */
typedef struct
{
    TIM_TypeDef*    ceExTimx;                           /*!< Pwmʹ�õ�TIMx��Դ���û������ע*/
    GPIO_TypeDef*   ceExGpiox;                          /*!< Pwm��Ӧ��STM32F103���������û������ע*/
    uint16          ceExGpioPinx;                       /*!< Pwm��Ӧ��STM32F103���źţ��û������ע*/
    uint16          ceExTimPrescaler;
    uint16          ceExTimPeriod;
    uint16          ceExTimCCRx;
    uint32          ceExOldCycle;
    uint32          ceExOldDuty;
}CeExPwmPar;

/**
  * @brief  �ṹ�壬AD�����ڲ���չ���Լ���
  */
typedef struct
{
    GPIO_TypeDef*   ceExGpiox;                          /*!< Ad��Ӧ��STM32F103���������û������ע*/
    uint16          ceExGpioPinx;                       /*!< Ad��Ӧ��STM32F103���źţ��û������ע*/
    uint8           ceAdChannelx;                       /*!< Ad��Ӧ��STM32F103��ADͨ�����û������ע*/
}CeExAdPar;



/**
  * @brief  �ṹ�壬CCP�����ڲ���չ���Լ���
  */
typedef struct
{
    uint32          ceExOutCcpCnt;                      /*!< Ccp����Ĵ������û������ע*/

    TIM_TypeDef*    ceExTIMx;                           /*!< Ccp��Ӧ��STM32F103��TIMx���û������ע*/
}CeExCcpPar;

/**
  * @brief  �ṹ�壬DA�����ڲ���չ���Լ���
  */
typedef struct
{
    uint8                   ceExIsConvertFinish;        /*!< Daת���Ƿ���ɣ��û������ע*/
    DMA_Channel_TypeDef*    ceExDMAChannel;             /*!< Da��Ӧ��STM32F103��DMAͨ�����û������ע*/
    TIM_TypeDef*            ceExTIMx;                   /*!< Da��Ӧ��STM32F103��TIMx���û������ע*/
    uint32                  ceExDAC_Channel_x;          /*!< Da��Ӧ��STM32F103��DA_DMA���û������ע*/
    GPIO_TypeDef*           ceExGpiox;                  /*!< Da��Ӧ��STM32F103�����������û������ע*/
    uint16                  ceExGpioPinx;               /*!< Da��Ӧ��STM32F103�����źţ��û������ע*/
}CeExDaPar;

/**
  * @brief  �ṹ�壬GPIO�����ڲ���չ���Լ���
  */
typedef struct
{
    GPIO_TypeDef*   ceExGpiox;                          /*!< ��ӦSTM32F103�����������û������ע*/
    uint16          ceExGpioPinx;                       /*!< ��ӦSTM32F103�����ű�ţ��û������ע*/
}CeExGpioPar;

/**
  * @brief  �ṹ�壬I2cMaster�����ڲ���չ���Լ���
  */
typedef struct
{
    uint16              ceExDelayNs;                    /*!< I2C��ƽ������Сʱ�䣬�û������ע*/
    GPIO_TypeDef*       ceExSCLGpiox;                   /*!< I2C SCL���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExSCLGpioPinx;                /*!< I2C SCL���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    GPIO_TypeDef*       ceExSDAGpiox;                   /*!< I2C SDA���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExSDAGpioPinx;                /*!< I2C SDA���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    uint8*              ceExI2cMasterStatusx;           /*!< I2cʹ�õ�����״̬λ��ַ���û������ע*/
}CeExI2cMasterPar;

/**
  * @brief  �ṹ�壬�ⲿ�ж�Int�����ڲ���չ���Լ���
  */
typedef struct
{
    GPIO_TypeDef*       ceExGpiox;                      /*!< �ⲿ�ж�Int��Ӧ��STM32F103�����������û������ע*/
    uint16              ceExGpioPinx;                   /*!< �ⲿ�ж�Int��Ӧ��STM32F103�����źţ��û������ע*/
    uint8               ceExIsStart;                    /*!< �ⲿ�ж�Int�Ƿ��ڹ������У��û������ע*/
    NVIC_InitTypeDef    ceExNVIC_InitStructure;         /*!< �ⲿ�ж�Int�жϳ�ʼ�����ýṹ�壬�û������ע*/
    EXTI_InitTypeDef    ceExEXTI_InitStructure;
}CeExIntPar;

/**
  * @brief  �ṹ�壬SpiMaster�����ڲ���չ���Լ���
  */
typedef struct
{
    SPI_TypeDef*        ceExSPIx;                       /*!< SPI��Ӧ��STM32F103�ڲ���Դ���û������ע*/
    GPIO_TypeDef*       ceExNSSGpiox;                   /*!< SPI NSS���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExNSSGpioPinx;                /*!< SPI NSS���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    GPIO_TypeDef*       ceExSCKGpiox;                   /*!< SPI SCK���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExSCKGpioPinx;                /*!< SPI SCK���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    GPIO_TypeDef*       ceExMOSIGpiox;                  /*!< SPI MOSI���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExMOSIGpioPinx;               /*!< SPI MOSI���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    GPIO_TypeDef*       ceExMISOGpiox;                  /*!< SPI MISO���Ŷ�Ӧ��STM32F103���������û������ע*/
    uint16              ceExMISOGpioPinx;               /*!< SPI MISO���Ŷ�Ӧ��STM32F103���źţ��û������ע*/
    uint8*              ceExSpiMasterStatusx;           /*!< SPIʹ�õ�����״̬λ��ַ���û������ע*/
}CeExSpiMasterPar;

/**
  * @brief  �ṹ�壬Uart�����ڲ���չ���Լ���
  */
typedef struct
{
    DMA_Channel_TypeDef *   ceExDMAChannelTx;           /*!< Uart��ӦSTM32F103�ķ���DMAͨ����ţ��û������ע*/
    DMA_Channel_TypeDef *   ceExDMAChannelRx;           /*!< Uart��ӦSTM32F103�Ľ���DMAͨ����ţ��û������ע*/
    uint32                  ceExDMAx_FLAG_GLx_Tx;       /*!< Uart��ӦSTM32F103��DMA���ͱ�־���û������ע*/
    uint32                  ceExDMAx_FLAG_GLx_Rx;       /*!< Uart��ӦSTM32F103��DMA���ձ�־���û������ע*/
    uint8                   ceExDMAx_Channelx_IRQn;     /*!< Uart��ӦSTM32F103��UART_DMA�жϺţ��û������ע*/
    USART_TypeDef *         ceExUartx;                  /*!< Uart��ӦSTM32F103����Դ���û������ע*/
    uint8                   ceExIsSendFinishFlag;       /*!< Uart�����Ƿ���ɱ�־*/
}CeExUartPar;

/**
  * @brief  �ṹ�壬Tg�����ڲ���չ���Լ���
  */
typedef struct
{
    GPIO_TypeDef*   ceExGpiox0;                         /*!< Tg ��һ��GPIO��Ӧ�Ķ�Ӧ�����������û������ע*/
    uint16          ceExGpioPinx0;                      /*!< Tg ��һ��GPIO��Ӧ�Ķ�Ӧ�����ű�ţ��û������ע*/
    GPIO_TypeDef*   ceExGpiox1;                         /*!< Tg �ڶ���GPIO��Ӧ�Ķ�Ӧ�����������û������ע*/
    uint16          ceExGpioPinx1;                      /*!< Tg �ڶ���GPIO��Ӧ�Ķ�Ӧ�����ű�ţ��û������ע*/
    GPIO_TypeDef*   ceExGpiox2;                         /*!< Tg ������GPIO��Ӧ�Ķ�Ӧ�����������û������ע*/
    uint16          ceExGpioPinx2;                      /*!< Tg ������GPIO��Ӧ�Ķ�Ӧ�����ű�ţ��û������ע*/
} CeExTgPar;

/**
  * @brief  �ṹ�壬Timer�����ڲ���չ���Լ���
  */
typedef struct
{
    TIM_TypeDef*    ceExTimx;
    uint8           ceExIsStart;                        /*!< �߾��ȶ�ʱ��Timer�Ƿ��ڹ������У��û������ע*/
    uint16          ceExTimPrescaler;
    uint16          ceExTimPeriod;
    uint64          ceExIntOccCnt;                      /*!< ��ʱ����start��ʼ���������������жϴ���*/
} CeExTimerPar;

/**
  * @brief  �ṹ�壬Ticker�����ڲ���չ���Լ���
  */
typedef struct
{
    uint32      nowTick;                                /*!< ��ǰ��������*/
    uint8       isRunning;                              /*!< �˶�ʱ�����Ƿ���������*/
}CeExTickerPar;

/**
  * @brief  �ṹ�壬Task�����ڲ���չ���Լ���
  */
typedef struct
{
    uint8   isRunning;                                  /*!< ָʾ��ǰ�߳��Ƿ���������*/
    #ifdef __CE_USE_RTOS__
    uint8   ceExTaskPriority;
    #endif
}CeExTaskPar;

#ifdef __CE_CHECK_PAR__
extern void ce_assert_failed(uint8_t* file, uint32_t line,CE_STATUS ceStatus);              /*!< �������г�������ݴ˺�����ӡ������Ϣ*/
#endif //__CE_CHECK_PAR__

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_MCU_H__
