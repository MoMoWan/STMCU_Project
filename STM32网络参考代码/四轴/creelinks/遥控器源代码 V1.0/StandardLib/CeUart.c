/**
  ******************************************************************************
  * @file   CeUart.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2016-08-05
  * @brief  ����STM32F103RET6������ƽ̨��CeUart��Դ����ʵ�ֿ��ļ�
  ******************************************************************************
  * @attention
  *
  *1)���Դ��ļ������ø���Uart��Դ���ж����ȼ����й����ȼ���������ο�STM32F103RET6�ֲ�
  *2)����ʹ��DMA���䷽ʽ����ҪΪDMA�ṩһ�����������������ݣ��û��豣֤Uartͨ��ÿ���������յ������ݰ�����С��
  *  CE_UART_Rx_DMA_BUF_SIZE��Ĭ��Ϊ512�������Ҫ���û����޸�Ϊ�����ֵ��
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeUart.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

const uint16 ceUartIntPriority[] = {0x2131,0x2131,0x2131,0x2121,0x2121};/*!< ��ӦUart��������յ��ж����ȼ���¼0x2131��ʾ�����ж���ռʽ���ȼ�Ϊ2����Ӧʽ���ȼ�Ϊ1�����յ���ռʽ���ȼ�Ϊ3����Ӧʽ���ȼ�Ϊ1*/

#define CE_UART1_DMA_BUF_SIZE            512 /*!< ��ԴR14DMAͨ��ʹ�õĻ��泤��*/
#define CE_UART2_DMA_BUF_SIZE            512 /*!< ��ԴR24DMAͨ��ʹ�õĻ��泤��*/
#define CE_UART3_DMA_BUF_SIZE            512 /*!< ��ԴR9DMAͨ��ʹ�õĻ��泤��*/
#define CE_UART4_DMA_BUF_SIZE            0   /*!< ��ԴR31δʹ��DMA��ʽ���䣬�ʲ���Ҫ�ṩDMA����*/
#define CE_UART5_DMA_BUF_SIZE            0   /*!< ��ԴR18δʹ��DMA��ʽ���䣬�ʲ���Ҫ�ṩDMA����*/

typedef struct
{
    uint16 txIndex;
    uint16 rxIndex;
    uint8* sendBuf;
    uint32 sendBufSize;
} CeUartBase;

CeUartBase ceUartBase4;    /*!< Uart4δʹ��DMA�����ﾴ�ضԴ������ж��η�װ*/
CeUartBase ceUartBase5;    /*!< Uart5δʹ��DMA�����ﾴ�ضԴ������ж��η�װ*/


CeUart* ceUartList[]={CE_NULL,CE_NULL,CE_NULL,CE_NULL,CE_NULL};      /*!< ���ڱ���STM32F103��Ӧ��5��Uart���Զ���*/
uint8 ceUartDmaTemp[1];

uint8 ceUart1DmaBuf[CE_UART1_DMA_BUF_SIZE];
uint8 ceUart2DmaBuf[CE_UART2_DMA_BUF_SIZE];
uint8 ceUart3DmaBuf[CE_UART3_DMA_BUF_SIZE];


/**
  * @brief   Uart1�Ľ��տ����жϣ������յ�һ���������ݺ�һ��ʱ��û�������ݱ��󻬼��أ��򴥷����ж�
  * @param   None
  * @return  None
  */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)// �����ж�
    {
        uint32 recvCount = ceUartList[0]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA_Cmd(DMA1_Channel5, DISABLE);// �ر�DMA ����ֹ����
        DMA_ClearFlag( DMA1_FLAG_GL5);// ��DMA��־λ
        DMA1_Channel5->CNDTR = ceUartList[0]->recvBufSize;
        ceFifoOp.write(&(ceUartList[0]->ceExFifo),ceUart1DmaBuf,recvCount);//Uart�ڲ�ʹ�õ�Fifo�����Ա�֤ͬһʱ��ֻ��һ���߳���д

        USART_ReceiveData( USART1);         // Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}

/**
  * @brief   Uart2�Ľ��տ����жϣ������յ�һ���������ݺ�һ��ʱ��û�������ݱ��󻬼��أ��򴥷����ж�
  * @param   None
  * @return  None
  */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)// �����ж�
    {
        uint32 recvCount = ceUartList[1]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel6);
        DMA_Cmd(DMA1_Channel6, DISABLE);// �ر�DMA ����ֹ����
        DMA_ClearFlag( DMA1_FLAG_GL6);// ��DMA��־λ
        DMA1_Channel6->CNDTR = ceUartList[1]->recvBufSize;

        ceFifoOp.write(&(ceUartList[1]->ceExFifo),ceUart2DmaBuf,recvCount);//Uart�ڲ�ʹ�õ�Fifo�����Ա�֤ͬһʱ��ֻ��һ���߳���д

        USART_ReceiveData(USART2);// Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel6, ENABLE);
    }
}

/**
  * @brief   Uart3�Ľ��տ����жϣ������յ�һ���������ݺ�һ��ʱ��û�������ݱ��󻬼��أ��򴥷����ж�
  * @param   None
  * @return  None
  */
void USART3_IRQHandler(void)
{

    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)// �����ж�
    {
        uint32 recvCount = ceUartList[2]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel3);
        DMA_Cmd(DMA1_Channel3, DISABLE);// �ر�DMA ����ֹ����
        DMA_ClearFlag( DMA1_FLAG_GL3);// ��DMA��־λ
        DMA1_Channel3->CNDTR = ceUartList[2]->recvBufSize;
        ceFifoOp.write(&(ceUartList[2]->ceExFifo),ceUart3DmaBuf,recvCount);//Uart�ڲ�ʹ�õ�Fifo�����Ա�֤ͬһʱ��ֻ��һ���߳���д
        USART_ReceiveData(USART3);// Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel3, ENABLE);
    }

}

/**
  * @brief   Uar4�ķ�����ɣ����յ������жϺ�����δʹ��Dma��ʽ���ʽ��Ƽ�����Debug���Դ���
  * @param   None
  * @return  None
  */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_TC) != RESET)// Transmit the string in a loop
    {
        if (ceUartBase4.txIndex >= ceUartBase4.sendBufSize)
        {
            USART_ITConfig(UART4, USART_IT_TC, DISABLE);
            ceUartList[3]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
            ceUartBase4.txIndex = 0;
        } else
        {
            USART_SendData(UART4, ceUartBase4.sendBuf[ceUartBase4.txIndex]);
            ceUartBase4.txIndex++;
        }
    }

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)// Received characters modify string
    {
        uint8 temp = USART_ReceiveData(UART4);
        //ceUartList.ceUart4->recvBuf[ceUart4Base.rxIndex] = USART_ReceiveData(UART4);
        //ceUart4Base.rxIndex++;
        ceFifoOp.write(&(ceUartList[3]->ceExFifo), &temp, 1);//Uart�ڲ�ʹ�õ�Fifo�����Ա�֤ͬһʱ��ֻ��һ���߳���д
    }

    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)// �����ж�
    {
        USART_ReceiveData(UART4);// Clear IDLE interrupt flag bit
    }
}

/**
  * @brief   Uart5�ķ�����ɣ����յ������жϺ�����δʹ��Dma��ʽ���ʽ��Ƽ�����Debug���Դ���
  * @param   None
  * @return  None
  */
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_TC) != RESET)// Transmit the string in a loop
    {
        if (ceUartBase5.txIndex >= ceUartBase5.sendBufSize)
        {
            USART_ITConfig(UART5, USART_IT_TC, DISABLE);
            ceUartList[4]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
            ceUartBase5.txIndex = 0;
        } else
        {
            USART_SendData(UART5, ceUartBase5.sendBuf[ceUartBase5.txIndex]);
            ceUartBase5.txIndex++;
        }
    }

    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)// Received characters modify string
    {
        uint8 temp = USART_ReceiveData(UART5);
        //ceUartList.ceUart5->recvBuf[ceUart5Base.rxIndex] = USART_ReceiveData(UART5);
        //ceUart5Base.rxIndex++;
        ceFifoOp.write(&(ceUartList[4]->ceExFifo), &temp, 1);//Uart�ڲ�ʹ�õ�Fifo�����Ա�֤ͬһʱ��ֻ��һ���߳���д
    }

    if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)// �����ж�
    {
        USART_ReceiveData(UART5);// Clear IDLE interrupt flag bit
    }
}

/**
  * @brief   DMA1,CH4ͨ����������жϺ�������ӦUart1�ķ������
  * @param   None
  * @return  None
  */
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag( DMA1_FLAG_GL4);// �����־
        DMA_Cmd(DMA1_Channel4, DISABLE);// �ر�DMAͨ��
    }
    ceUartList[0]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}

/**
  * @brief   DMA1,CH7ͨ����������жϺ�������ӦUart2�ķ������
  * @param   None
  * @return  None
  */
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        DMA_ClearFlag( DMA1_FLAG_GL7);// �����־
        DMA_Cmd(DMA1_Channel7, DISABLE);// �ر�DMAͨ��
    }
    ceUartList[1]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}


/**
  * @brief   DMA1,CH2ͨ����������жϺ�������ӦUart3�ķ������
  * @param   None
  * @return  None
  */
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC2))
    {
        DMA_ClearFlag(DMA1_FLAG_GL2);// �����־
        DMA_Cmd(DMA1_Channel2, DISABLE);// �ر�DMAͨ��
    }
    ceUartList[2]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}


#ifdef __CE_CHECK_PAR__
/**
  * @brief   ����Uartָ�����
  * @param   ceUart:CeUart���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeUart(CeUart* ceUart)
{
    if (ceUart == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if ((ceUart->ceResource & CE_RES_MARK_UART) != CE_RES_MARK_UART)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   ��ʼ��Uart
  * @param   ceUart:CeUart���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS��CE_STATUS_RESOURCE_ERROR��CE_STATUS_NULL_POINTER
  */
CE_STATUS ceUart_initial(CeUart* ceUart)
{
    GPIO_InitTypeDef GPIO_InitStructureTx;
    GPIO_InitTypeDef GPIO_InitStructureRx;
    NVIC_InitTypeDef NVIC_InitStructureTx;
    NVIC_InitTypeDef NVIC_InitStructureRx;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_TypeDef * GPIOx;
    uint8 UART_IRQN;
    uint8 uartIndex;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__

    uartIndex = ceUart->ceResource & 0x0000000F;

    ceUart->ceExFifo.buff = ceUart->recvBuf;
    ceUart->ceExFifo.buffSize = ceUart->recvBufSize;
    ceFifoOp.initial(&(ceUart->ceExFifo));

    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x00;   //��ֹδ��ȫ��ʼ���㿪ʼ��������

    switch (ceUart->ceResource)
    {
    case Uart1://Uart1
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_10;
        ceUart->ceExUartPar.ceExUartx = USART1;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel4;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel5;
        UART_IRQN = USART1_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL4;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL5;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel4_IRQn;
        ceUartList[0] = ceUart;
        GPIOx = GPIOA;
        break;
    case Uart2://Uart2
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_3;
        ceUart->ceExUartPar.ceExUartx = USART2;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel7;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel6;
        UART_IRQN = USART2_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL7;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL6;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel7_IRQn;
        ceUartList[1] = ceUart;
        GPIOx = GPIOA;
        break;
    case Uart3://Uart3
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_11;
        ceUart->ceExUartPar.ceExUartx = USART3;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel2;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel3;
        UART_IRQN = USART3_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL2;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL3;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel2_IRQn;
        ceUartList[2] = ceUart;
        GPIOx = GPIOB;
        break;
    case Uart4://Uart4
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_11;
        ceUart->ceExUartPar.ceExUartx = UART4;
        UART_IRQN = UART4_IRQn;
        ceUartList[3] = ceUart;
        GPIOx = GPIOC;
        break;
    case Uart5://Uart5
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_2;
        ceUart->ceExUartPar.ceExUartx = UART5;
        UART_IRQN = UART5_IRQn;// ���պͷ����ж�
        ceUartList[4] = ceUart;
        GPIOx = GPIOC;
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    NVIC_InitStructureTx.NVIC_IRQChannel = ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn;//����DMAͨ�����ж�����
    NVIC_InitStructureRx.NVIC_IRQChannel = UART_IRQN;//���ڷ��ͻ�����ж�����
    NVIC_InitStructureTx.NVIC_IRQChannelPreemptionPriority = (ceUartIntPriority[uartIndex]>>12)&0x000F;//���ȼ�����
    NVIC_InitStructureTx.NVIC_IRQChannelSubPriority = (ceUartIntPriority[uartIndex]>>8)&0x000F;
    NVIC_InitStructureRx.NVIC_IRQChannelPreemptionPriority = (ceUartIntPriority[uartIndex]>>4)&0x000F;
    NVIC_InitStructureRx.NVIC_IRQChannelSubPriority = (ceUartIntPriority[uartIndex]>>0)&0x000F;
    NVIC_InitStructureTx.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureTx);//TX

    NVIC_InitStructureRx.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureRx);//RX

    GPIO_InitStructureTx.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureTx.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructureTx);
    GPIO_InitStructureRx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init((UART_IRQN == UART5_IRQn ? GPIOD : GPIOx), &GPIO_InitStructureRx);//ע�⣬ֻ��Uart5��Tx��Rx�˿ڲ���һ���ģ�

    if (UART_IRQN != UART4_IRQn && UART_IRQN != UART5_IRQn)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);// ����DMA1ʱ��

        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, DISABLE);                 // ��DMAͨ��
        DMA_DeInit(ceUart->ceExUartPar.ceExDMAChannelTx);                       // �ָ�ȱʡֵ
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) (&ceUart->ceExUartPar.ceExUartx->DR);// ���ô��ڷ������ݼĴ���
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) ceUartDmaTemp;          // ���÷��ͻ������׵�ַ
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // ��������λĿ�꣬�ڴ滺���� ->����Ĵ���
        DMA_InitStructure.DMA_BufferSize = (uint16) 1;                          // ��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ滺������ַ���ӵ���
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ��8λ��1���ֽ�
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ��8λ��1���ֽ�
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ���δ���ģʽ
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // �ر��ڴ浽�ڴ��DMAģʽ
        DMA_Init(ceUart->ceExUartPar.ceExDMAChannelTx, &DMA_InitStructure);     // д������
        DMA_ClearFlag(ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx);                // ���DMA���б�־
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, DISABLE);                 // �ر�DMA
        DMA_ITConfig(ceUart->ceExUartPar.ceExDMAChannelTx, DMA_IT_TC, ENABLE);  // ��������DMAͨ���ж�

        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelRx, DISABLE);                 // ��DMAͨ��
        DMA_DeInit(ceUart->ceExUartPar.ceExDMAChannelRx);                       // �ָ�ȱʡֵ
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) (&ceUart->ceExUartPar.ceExUartx->DR);// ���ô��ڽ������ݼĴ���
        switch (UART_IRQN)
        {
        case USART2_IRQn:
             DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart2DmaBuf;      // ���ý��ջ������׵�ַ
            break;
        case USART1_IRQn:
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart1DmaBuf;       // ���ý��ջ������׵�ַ
            break;
        case USART3_IRQn:
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart3DmaBuf;       // ���ý��ջ������׵�ַ
            break;
        default:
            break;
        }

        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // ��������Ϊ����Դ������Ĵ��� -> �ڴ滺����
        DMA_InitStructure.DMA_BufferSize = (uint16)ceUart->recvBufSize;        // ��Ҫ�����ܽ��յ����ֽ���
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ滺������ַ���ӵ���
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ��8λ��1���ֽ�
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ��8λ��1���ֽ�
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ���δ���ģʽ
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // �ر��ڴ浽�ڴ��DMAģʽ
        DMA_Init(ceUart->ceExUartPar.ceExDMAChannelRx, &DMA_InitStructure);     // д������
        DMA_ClearFlag(ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx);                // ���DMA���б�־
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelRx, ENABLE);                  // ��������DMAͨ�����ȴ���������
    }
    USART_InitStructure.USART_BaudRate = ceUart->uartBaudRate;
    USART_InitStructure.USART_WordLength = ceUart->uartWordLength;
    USART_InitStructure.USART_WordLength = ceUart->uartWordLength == CE_UART_WORD_LENGTH_9B ? USART_WordLength_9b : USART_WordLength_8b;
    switch (ceUart->uartStopBits)
    {
    case CE_UART_STOP_BITS_1:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    case CE_UART_STOP_BITS_0_5:
        USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
        break;
    case CE_UART_STOP_BITS_2:
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        break;
    case CE_UART_STOP_BITS_1_5:
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
        break;
    default:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    }
    switch(ceUart->uartParity)
    {
    case CE_UART_PARITY_NO:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    case CE_UART_PARITY_EVEN:
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        break;
    case CE_UART_PARITY_ODD:
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        break;
    default:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(ceUart->ceExUartPar.ceExUartx, &USART_InitStructure);

    USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_IDLE, ENABLE);
    if (UART_IRQN != UART4_IRQn && UART_IRQN != UART5_IRQn)
    {
        USART_DMACmd(ceUart->ceExUartPar.ceExUartx, USART_DMAReq_Tx, ENABLE);
        USART_DMACmd(ceUart->ceExUartPar.ceExUartx, USART_DMAReq_Rx, ENABLE);
    }
    else
    {
        //USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TXE, ENABLE);//�ڽ��н��յ�ʱ���ٿ�����Ӧ��TX�ж�
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_RXNE, ENABLE);
    }
    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x01;

    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ʼUart
  * @param   eUart:ceUart���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceUart_start(CeUart* ceUart)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    USART_Cmd(ceUart->ceExUartPar.ceExUartx, ENABLE);  // ��������
}

/**
  * @brief   ͨ��Uart���������ݣ�DMA��ʽ
  * @param   ceUart:CeUart���Զ���ָ��
  * @param   dataBuf:�����͵�����
  * @param   dataCount:�����͵����ݳ���
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
CE_STATUS ceUart_sendData(CeUart* ceUart, uint8* dataBuf, uint16 dataCount)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    if (dataCount == 0)
    {
        return CE_STATUS_SUCCESS;
    }
    while (ceUart->ceExUartPar.ceExIsSendFinishFlag == 0x00);
    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x00;
    while (USART_GetFlagStatus(ceUart->ceExUartPar.ceExUartx, USART_FLAG_TC) == RESET);

    if (ceUart->ceResource == Uart4)
    {
        ceUartBase4.sendBuf = dataBuf;
        ceUartBase4.sendBufSize = dataCount;
        ceUartBase4.rxIndex = 0;
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TC, ENABLE);
    }
    else if (ceUart->ceResource == Uart5)
    {
        ceUartBase5.sendBuf = dataBuf;
        ceUartBase5.sendBufSize = dataCount;
        ceUartBase5.rxIndex = 0;
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TC, ENABLE);
    }
    else
    {
        ceUart->ceExUartPar.ceExDMAChannelTx->CMAR = (uint32)dataBuf;
        ceUart->ceExUartPar.ceExDMAChannelTx->CNDTR = (uint16)dataCount;// ����Ҫ���͵��ֽ���Ŀ
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, ENABLE);//��ʼDMA����
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   ��ý��ջ����еĿ�������
  * @param   ceUart:CeUart���Զ���ָ��
  * @return  ���ؿɶ�ȡ�����ݳ���
  */
uint16 ceUart_getRecvDataCount(CeUart* ceUart)
{
    return ceFifoOp.getCanReadSize(&(ceUart->ceExFifo));
}

/**
  * @brief   ��������
  * @param   ceUart:CeUart���Զ���ָ��
  * @return  ����ʵ�ʶ�ȡ�������ݳ���
  */
uint16 ceUart_readData(CeUart* ceUart, uint8* dataBuf, uint16 readCount)
{
    return ceFifoOp.read(&(ceUart->ceExFifo),dataBuf, readCount);
}

/**
  * @brief   ֹͣUart
  * @param   ceUart:CeUart���Զ���ָ��
  * @return  ϵͳ״̬�룬���ܵ�ֵ:CE_STATUS_SUCCESS
  */
void ceUart_stop(CeUart* ceUart)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    USART_Cmd(ceUart->ceExUartPar.ceExUartx, DISABLE);
}

void ceUart_clearRecvBuf(CeUart* ceUart)
{
    ceFifoOp.clear(&(ceUart->ceExFifo));
}

const CeUartOp ceUartOp = {ceUart_initial, ceUart_start, ceUart_sendData, ceUart_getRecvDataCount, ceUart_readData, ceUart_stop, ceUart_clearRecvBuf};

#ifdef __cplusplus
 }
#endif //__cplusplus
