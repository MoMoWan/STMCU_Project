#include "BSP_FDC2214.h"

uint32_t Data_FDC;

void I2C_Read_Data(I2C_HandleTypeDef *hi2c, uint8_t *Buf, uint8_t len)
{
}
/*IIC写2个字节 
 *reg:寄存器地址
 *data1:数据1
 *data2:数据2
 *返回值:0    正常
 *     其他  错误代码
*/
uint8_t FDC2214_WriteReg(uint8_t reg, uint8_t MSB, uint8_t LSB)
{
	uint8_t data[2];
	data[0] = MSB;
	data[1] = LSB;
	HAL_I2C_Master_Transmit(&hFDC2214, FDC2214_ADDR << 1, data, 2, 0xFF);
}

/*读取FDC2214寄存器数据
 *IIC读2个字节 
 *reg:寄存器地址 
 *返回值:读到的数据
 */
uint16_t FDC2214_ReadReg(uint8_t Reg)
{
	uint16_t data;
	uint8_t Buf[2];
	HAL_I2C_Master_Receive(&hFDC2214, FDC2214_ADDR << 1, Buf, 2, 0xFF);
	data = Buf[0] << 16 | Buf[1];
	return data;
}

/*FDC2214初始化函数
*返回值:0：初始化正常
*       1：不正常
*/
uint8_t FDC2214_Init(void)
{
	uint16_t res;

	res = FDC2214_ReadReg(FDC2214_MANUFACTURER_ID);
	if (res == 0x5449)
	{
		//设置FDC2214_WriteReg寄存器
		FDC2214_WriteReg(FDC2214_RCOUNT_CH0, 0x34, 0xFB); //参考计数转换间隔时间（T=(RCOUNT_CH0*16)/Frefx）
		FDC2214_WriteReg(FDC2214_RCOUNT_CH1, 0x34, 0xFB);
		FDC2214_WriteReg(FDC2214_RCOUNT_CH2, 0x34, 0xFB);

		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH0, 0x00, 0x1B); //转换之前的稳定时间（T=(SETTLECOUNT_CHx*16)/Frefx）
		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH1, 0x00, 0x1B);
		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH2, 0x00, 0x1B);

		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH0, 0x20, 0x02); //选择在0.01MHz ~ 10MHz的传感器频率
		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH1, 0x20, 0x02); //Frefx = Fclk = 43.4MHz/2(2分频)
		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH2, 0x20, 0x02); //CHx_REF_DIVIDER=2;CHx_FIN_SEL=2

		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH0, 0x78, 0x00); //0.146ma（传感器时钟建立+转换时间的驱动电流）
		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH1, 0x78, 0x00);
		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH2, 0x78, 0x00);

		FDC2214_WriteReg(FDC2214_ERROR_CONFIG, 0x00, 0x00); //全部禁止错误汇报

		FDC2214_WriteReg(FDC2214_MUX_CONFIG, 0xC2, 0x0D); //通道0，1，2 ，3；选择10Mhz为超过振荡槽振荡频率的最低设置，多通道，四通道

		FDC2214_WriteReg(FDC2214_CONFIG, 0x14, 0x01); //激活模式，使用内部振荡器做参考频率，INTB引脚会随状态寄存器更新被置位
	}
	else
		return 1;

	return 0;
}

uint32_t FCD2214_GetCap_Data(uint8_t CH) //数据分辨力28-bits
{
	uint32_t result;
	switch (CH)
	{
	case 0:
		result = FDC2214_ReadReg(FDC2214_DATA_CH0) & 0x0FFF;
		result = (result << 16) | (FDC2214_ReadReg(FDC2214_DATA_LSB_CH0));
		break;
	case 1:
		result = FDC2214_ReadReg(FDC2214_DATA_CH1) & 0x0FFF;
		result = (result << 16) | (FDC2214_ReadReg(FDC2214_DATA_LSB_CH1));
		break;
	case 2:
		result = FDC2214_ReadReg(FDC2214_DATA_CH2) & 0x0FFF;
		result = (result << 16) | (FDC2214_ReadReg(FDC2214_DATA_LSB_CH2));
		break;
	case 3:
		result = FDC2214_ReadReg(FDC2214_DATA_CH3) & 0x0FFF;
		result = (result << 16) | (FDC2214_ReadReg(FDC2214_DATA_LSB_CH3));
		break;
	default:
		break;
	}
	result = result & 0x0FFFFFFF;
	return result;
}

/*测量线路电容
 *index:0：线路0
 *      1：线路1
 *      2：线路2
 *      3：线路3
 *返回值：线路总电容C
 */
double FDC2214_Calculate_Cap(uint32_t Data)
{
	double Cap;

	//	Cap = 56645.763f/((float)Data_FDC);
	//	return ((Cap*Cap)-33);
	Cap = 232021045.248 / (Data);
	return (Cap * Cap);
}
