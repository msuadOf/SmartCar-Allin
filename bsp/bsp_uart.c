#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_UART.h"
#include "STC32G_NVIC.h"
#include "STC32G_DMA.h"
#include "STC32G_Switch.h"

#define DMA_Buffer_len 6
u8 xdata DmaBuffer[DMA_Buffer_len]; // �շ����û��棬ͬʱʹ�ö�·����ʱÿ��������ֱ��建�棬�����໥����
/******************** DMA ���� ********************/
void DMA_UART_config(void)
{

	DMA_UART_InitTypeDef DMA_UART_InitStructure; // �ṹ����

	DMA_UART_InitStructure.DMA_TX_Length = DMA_Buffer_len - 1; // DMA�������ֽ���  	(0~65535) + 1
	DMA_UART_InitStructure.DMA_TX_Buffer = (u16)DmaBuffer;	   // �������ݴ洢��ַ
	//	DMA_UART_InitStructure.DMA_RX_Length = 255;				//DMA�������ֽ���  	(0~65535) + 1
	//	DMA_UART_InitStructure.DMA_RX_Buffer = (u16)DmaBuffer;	//�������ݴ洢��ַ
	DMA_UART_InitStructure.DMA_TX_Enable = ENABLE; // DMAʹ��  	ENABLE,DISABLE
	//	DMA_UART_InitStructure.DMA_RX_Enable = ENABLE;		//DMAʹ��  	ENABLE,DISABLE
	//	DMA_UART_Inilize(UART1, &DMA_UART_InitStructure);	//��ʼ��
	DMA_UART_Inilize(UART2, &DMA_UART_InitStructure); // ��ʼ��
	//	DMA_UART_Inilize(UART3, &DMA_UART_InitStructure);	//��ʼ��
	//	DMA_UART_Inilize(UART4, &DMA_UART_InitStructure);	//��ʼ��

	//	NVIC_DMA_UART1_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART1_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	NVIC_DMA_UART2_Tx_Init(ENABLE, Priority_2, Priority_2); // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	NVIC_DMA_UART2_Rx_Init(ENABLE, Priority_0, Priority_0); // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3

	//	DMA_UR1R_CLRFIFO();		//��� DMA FIFO
	DMA_UR2R_CLRFIFO(); // ��� DMA FIFO
						//	DMA_UR3R_CLRFIFO();		//��� DMA FIFO
						//	DMA_UR4R_CLRFIFO();		//��� DMA FIFO
}

static void GPIO_config(void)
{
	P4_MODE_IO_PU(GPIO_Pin_6 | GPIO_Pin_7); // P4.6,P4.7 ����Ϊ׼˫��� - UART2
											// P2_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);
}
void UART_config(void)
{
	COMx_InitDefine COMx_InitStructure; // �ṹ����

	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // ģʽ,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use = BRT_Timer2;  // ѡ�����ʷ�����, BRT_Timer1, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2)
	COMx_InitStructure.UART_BaudRate = 115200ul;   // ������,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable = ENABLE;	   // ��������,   ENABLE��DISABLE
	//	UART_Configuration(UART1, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	UART_Configuration(UART2, &COMx_InitStructure); // ��ʼ������ UART1,UART2,UART3,UART4
	NVIC_UART2_Init(ENABLE, Priority_0);			// �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART3, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART3_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART4, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART4_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	//	UART1_SW(UART1_SW_P30_P31);		//UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
								//	UART3_SW(UART3_SW_P00_P01);		//UART3_SW_P00_P01,UART3_SW_P50_P51
								//	UART4_SW(UART4_SW_P02_P03);		//UART4_SW_P02_P03,UART4_SW_P52_P53
}
/****************  ���ڳ�ʼ������ *****************/
void bsp_uart_init(void)
{
	DMA_UART_config();
	UART_config();
	GPIO_config();
}
//DMA_UR2T_TRIG();