#include "BSP_NRF24L01.h"
/*
#define hNRF hspix  �ڶ�Ӧ��Ӳ�����壨SPI.h�ļ���
*/
static uint8_t TX_ADDRESS[5] = {0x1A, 0x3B, 0x5C, 0x7D, 0x9E}; //���ص�ַ
static uint8_t RX_ADDRESS[5] = {0x1A, 0x3B, 0x5C, 0x7D, 0x9E}; //���յ�ַ

/**
  * ��������: ������Flash��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
  * �������: byte������������
  * �� �� ֵ: uint8_t�����յ�������
  * ˵    ������
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
  uint8_t d_read, d_send = byte;
  if (HAL_SPI_TransmitReceive(hspi, &d_send, &d_read, 1, 0xFF) != HAL_OK)
  {
    d_read = 0xFF;
  }
  return d_read;
}

/**
  * ��������: SPIд�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:ָ���Ĵ�����ַ
  *           
  */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
  uint8_t status;
  NRF24L01_SPI_CS_ENABLE();                //ʹ��SPI����
  status = SPIx_ReadWriteByte(&hNRF, reg); //���ͼĴ�����
  SPIx_ReadWriteByte(&hNRF, value);        //д��Ĵ�����ֵ
  NRF24L01_SPI_CS_DISABLE();               //��ֹSPI����
  return (status);                         //����״ֵ̬
}

/**
  * ��������: ��ȡSPI�Ĵ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:Ҫ���ļĴ���
  *           
  */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
  uint8_t reg_val;
  NRF24L01_SPI_CS_ENABLE();                  //ʹ��SPI����
  SPIx_ReadWriteByte(&hNRF, reg);            //���ͼĴ�����
  reg_val = SPIx_ReadWriteByte(&hNRF, 0XFF); //��ȡ�Ĵ�������
  NRF24L01_SPI_CS_DISABLE();                 //��ֹSPI����
  return (reg_val);                          //����״ֵ̬
}

/**
  * ��������: ��ָ��λ�ö���ָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: �˴ζ�����״̬�Ĵ���ֵ 
  * ˵    ������
  *           
  */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status, uint8_t_ctr;

  NRF24L01_SPI_CS_ENABLE();                //ʹ��SPI����
  status = SPIx_ReadWriteByte(&hNRF, reg); //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(&hNRF, 0XFF); //��������
  }
  NRF24L01_SPI_CS_DISABLE(); //�ر�SPI����
  return status;             //���ض�����״ֵ̬
}

/**
  * ��������: ��ָ��λ��дָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:�Ĵ���(λ��)  *pBuf:����ָ��  len:���ݳ���
  *           
  */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status, uint8_t_ctr;
  NRF24L01_SPI_CS_ENABLE();                //ʹ��SPI����
  status = SPIx_ReadWriteByte(&hNRF, reg); //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hNRF, *pBuf++); //д������
  }
  NRF24L01_SPI_CS_DISABLE(); //�ر�SPI����
  return status;             //���ض�����״ֵ̬
}

/**
  * ��������: ���24L01�Ƿ����
  * �������: ��
  * �� �� ֵ: 0���ɹ�;1��ʧ��
  * ˵    ������          
  */
uint8_t NRF24L01_Check(void)
{
  uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5}; //0xA5  0xC2
  uint8_t i;

  NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //д��5���ֽڵĵ�ַ.
  NRF24L01_Read_Buf(TX_ADDR, buf, 5);                  //����д��ĵ�ַ
  for (i = 0; i < 5; i++)
    if (buf[i] != 0XA5)
      break;
  if (i != 5)
    return 1; //���24L01����
  return 0;   //��⵽24L01
}

/******************************************************************************
����ԭ�ͣ�	void NRF24L01_Init(uint8_t Channal,uint8_t Mode)
��    �ܣ�	NRF24L01��ʼ��
��    ����	Chanal 40��RFͨ��
Mode   1:TX  2:RX  
*******************************************************************************/

void NRF24L01_Init(uint8_t Channal, uint8_t Mode)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(FLUSH_TX, 0xff); //��շ��ͻ�����
  NRF24L01_Write_Reg(FLUSH_RX, 0xff); //��ս��ջ�����

  NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, 5);    //дTX�ڵ��ַ
  NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, 5); //дRX�ڵ��ַ

  NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      //ʹ��ͨ��0���Զ�Ӧ��
  NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  //ʹ��ͨ��0�Ľ��յ�ַ
  NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //�����Զ��ط����ʱ��:500us;����Զ��ط�����:10��
  NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, Channal);   //����RFͨ��ΪCHANAL
  NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, 32);     //����ͨ��0����Ч���ݿ��
  NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);   //����TX�������,0db����,2Mbps,���������濪��

  if (Mode == 1)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); //����
  else if (Mode == 2)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); //����

  NRF24L01_CE_HIGH(); //CEΪ��,�������ģʽ
}

/**
  * ��������: �ú�����ʼ��NRF24L01��RXģʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC
  NRF24L01_CE_HIGH();                               //CEΪ��,�������ģʽ
}

/**
  * ��������: �ú�����ʼ��NRF24L01��TXģʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */
void NRF24L01_TX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_CE_HIGH();                               //CEΪ��,10us����������
}

/**
  * ��������: ����NRF24L01����һ������
  * �������: ��
  * �� �� ֵ: �������״��
  * ˵    ����txbuf:�����������׵�ַ
  *           
  */
void NRF24L01_TxPacket(uint8_t *txbuf)
{
  //	 uint8_t sta;

  NRF24L01_CE_LOW();
  NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //д���ݵ�TX BUF  32���ֽ�
  NRF24L01_CE_HIGH();                                     //��������

  //	while( NRF24L01_IRQ_PIN_READ()!=0);//�ȴ��������
  //
  //	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
  //	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
  //	if(sta&MAX_TX)//�ﵽ����ط�����
  //	{
  //		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
  //		return MAX_TX;
  //	}
  //	if(sta&TX_OK)//�������
  //	{
  //		return TX_OK;
  //	}
  //	return 0xff;//����ԭ����ʧ��
}

/******************************************************************************
����ԭ�ͣ�	void NRF24L01_IRQ(void)
��    �ܣ�	NRF24L01�ж�
*******************************************************************************/
void NRF24L01_IRQ(uint8_t *txbuf)
{
  uint8_t status = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);

  if (status & (1 << RX_DR)) //�����ж�
  {
    uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID); //�յ����ݳ���
    if (rx_len == 32)
    {
      NRF24L01_Read_Buf(RD_RX_PLOAD, txbuf, rx_len); //��ȡ����FIFO����
    }
    else
    {
      NRF24L01_Write_Reg(FLUSH_RX, 0xff); //��ս��ջ�����
    }
  }
  if (status & (1 << MAX_RT)) //�ﵽ�����ط��ж�
  {
    if (status & (1 << TX_FULL)) //TX FIFO ���
    {
      NRF24L01_Write_Reg(FLUSH_TX, 0xff); //��շ��ͻ�����
    }
  }
  //	if(status & (1<<TX_DS))//�������
  //	{
  NRF24L01_RX_Mode();                                 //����Nrf2401Ϊ����ģʽ
                                                      //	}
  NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, status); //����жϱ�־λ
}

/**
  * ��������:����NRF24L01����һ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */
void NRF24L01_RxPacket(uint8_t *rxbuf)
{
  //	uint8_t sta;

  //	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
  //	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־

  //	if(sta&RX_OK)//���յ�����
  //	{
  //		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
  //		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
  //		return RX_OK;
  //	}
  //	return 1;//û�յ��κ�����
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
