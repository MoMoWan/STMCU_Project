#include "BSP_FDC2214.h"

uint32_t Data_FDC;

static HAL_StatusTypeDef I2Cx_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hFDC2214, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, 0xFF);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C2_ErrHandel();
  }        
  return status;
}

static HAL_StatusTypeDef I2Cx_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hFDC2214, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, 0xFF); 

/* Check the communication status */
  if(status != HAL_OK)
  {
    I2C2_ErrHandel();
  }        
  return status;
}

void FDC2214_Write(uint16_t DevAddress, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t Length)
{
  I2Cx_WriteBuffer(DevAddress, WriteAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, Length);
}

void FDC2214_Read(uint16_t DevAddress, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t Length)
{
  I2Cx_ReadBuffer(DevAddress, WriteAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, Length);
}
/*IICд2���ֽ� 
 *reg:�Ĵ�����ַ
 *data1:����1
 *data2:����2
 *����ֵ:0    ����
 *     ����  �������
*/
uint8_t FDC2214_WriteReg(uint8_t reg, uint8_t MSB, uint8_t LSB)
{
	uint8_t data[2];
	data[0] = MSB;
	data[1] = LSB;
	if (I2Cx_WriteBuffer( FDC2214_ADDR ,reg,I2C_MEMADD_SIZE_8BIT, data, 2) != HAL_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*��ȡFDC2214�Ĵ�������
 *IIC��2���ֽ� 
 *reg:�Ĵ�����ַ 
 *����ֵ:����������
 */
uint16_t FDC2214_ReadReg(uint8_t Reg)
{
	uint16_t data=0;
	uint8_t Buf[2]={0};
	FDC2214_Read(FDC2214_ADDR, Buf,Reg,1); 
	data = (Buf[0] << 8) | Buf[1];
	return data;
}

/*FDC2214��ʼ������
*����ֵ:0����ʼ������
*       1��������
*/
uint8_t FDC2214_Init(void)
{
	uint16_t MID=0,DID=0;

	MID = FDC2214_ReadReg(FDC2214_MANUFACTURER_ID);
	DID = FDC2214_ReadReg(FDC2214_DEVICE_ID);
	if (MID == 0x5449  && DID == 0x3055)
	{
		//����FDC2214_WriteReg�Ĵ���
		FDC2214_WriteReg(FDC2214_RCOUNT_CH0, 0x34, 0xFB); //�ο�����ת�����ʱ�䣨T=(RCOUNT_CH0*16)/Frefx��
		FDC2214_WriteReg(FDC2214_RCOUNT_CH1, 0x34, 0xFB);
		FDC2214_WriteReg(FDC2214_RCOUNT_CH2, 0x34, 0xFB);

		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH0, 0x00, 0x1B); //ת��֮ǰ���ȶ�ʱ�䣨T=(SETTLECOUNT_CHx*16)/Frefx��
		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH1, 0x00, 0x1B);
		FDC2214_WriteReg(FDC2214_SETTLECOUNT_CH2, 0x00, 0x1B);

		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH0, 0x20, 0x02); //ѡ����0.01MHz ~ 10MHz�Ĵ�����Ƶ��
		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH1, 0x20, 0x02); //Frefx = Fclk = 43.4MHz/2(2��Ƶ)
		FDC2214_WriteReg(FDC2214_CLOCK_DIVIDERS_C_CH2, 0x20, 0x02); //CHx_REF_DIVIDER=2;CHx_FIN_SEL=2

		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH0, 0x78, 0x00); //0.146ma��������ʱ�ӽ���+ת��ʱ�������������
		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH1, 0x78, 0x00);
		FDC2214_WriteReg(FDC2214_DRIVE_CURRENT_CH2, 0x78, 0x00);

		FDC2214_WriteReg(FDC2214_ERROR_CONFIG, 0x00, 0x00); //ȫ����ֹ����㱨

		FDC2214_WriteReg(FDC2214_MUX_CONFIG, 0xC2, 0x0D); //ͨ��0��1��2 ��3��ѡ��10MhzΪ�����񵴲���Ƶ�ʵ�������ã���ͨ������ͨ��

		FDC2214_WriteReg(FDC2214_CONFIG, 0x14, 0x01); //����ģʽ��ʹ���ڲ��������ο�Ƶ�ʣ�INTB���Ż���״̬�Ĵ������±���λ
	}
	else
	{
		return 1;
	}
	return 0;
}

uint32_t FCD2214_GetCap_Data(uint8_t CH) //���ݷֱ���28-bits
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

/*������·����
 *index:0����·0
 *      1����·1
 *      2����·2
 *      3����·3
 *����ֵ����·�ܵ���C
 */
double FDC2214_Calculate_Cap(uint32_t Data)
{
	double Cap;

	//	Cap = 56645.763f/((float)Data_FDC);
	//	return ((Cap*Cap)-33);
	Cap = 232021045.248 / (Data);
	return (Cap * Cap);
}
