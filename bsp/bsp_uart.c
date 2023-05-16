#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_UART.h"
#include "STC32G_NVIC.h"
#include "STC32G_DMA.h"
#include "STC32G_Switch.h"

#define DMA_Buffer_len 6
u8 xdata DmaBuffer[DMA_Buffer_len]; // 收发共用缓存，同时使用多路串口时每个串口需分别定义缓存，以免相互干扰
/******************** DMA 配置 ********************/
void DMA_UART_config(void)
{

	DMA_UART_InitTypeDef DMA_UART_InitStructure; // 结构定义

	DMA_UART_InitStructure.DMA_TX_Length = DMA_Buffer_len - 1; // DMA传输总字节数  	(0~65535) + 1
	DMA_UART_InitStructure.DMA_TX_Buffer = (u16)DmaBuffer;	   // 发送数据存储地址
	//	DMA_UART_InitStructure.DMA_RX_Length = 255;				//DMA传输总字节数  	(0~65535) + 1
	//	DMA_UART_InitStructure.DMA_RX_Buffer = (u16)DmaBuffer;	//接收数据存储地址
	DMA_UART_InitStructure.DMA_TX_Enable = ENABLE; // DMA使能  	ENABLE,DISABLE
	//	DMA_UART_InitStructure.DMA_RX_Enable = ENABLE;		//DMA使能  	ENABLE,DISABLE
	//	DMA_UART_Inilize(UART1, &DMA_UART_InitStructure);	//初始化
	DMA_UART_Inilize(UART2, &DMA_UART_InitStructure); // 初始化
	//	DMA_UART_Inilize(UART3, &DMA_UART_InitStructure);	//初始化
	//	DMA_UART_Inilize(UART4, &DMA_UART_InitStructure);	//初始化

	//	NVIC_DMA_UART1_Tx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	//	NVIC_DMA_UART1_Rx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	NVIC_DMA_UART2_Tx_Init(ENABLE, Priority_2, Priority_2); // 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	NVIC_DMA_UART2_Rx_Init(ENABLE, Priority_0, Priority_0); // 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Tx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Rx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Tx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Rx_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3

	//	DMA_UR1R_CLRFIFO();		//清空 DMA FIFO
	DMA_UR2R_CLRFIFO(); // 清空 DMA FIFO
						//	DMA_UR3R_CLRFIFO();		//清空 DMA FIFO
						//	DMA_UR4R_CLRFIFO();		//清空 DMA FIFO
}

static void GPIO_config(void)
{
	P4_MODE_IO_PU(GPIO_Pin_6 | GPIO_Pin_7); // P4.6,P4.7 设置为准双向口 - UART2
											// P2_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);
}
void UART_config(void)
{
	COMx_InitDefine COMx_InitStructure; // 结构定义

	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // 模式,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use = BRT_Timer2;  // 选择波特率发生器, BRT_Timer1, BRT_Timer2 (注意: 串口2固定使用BRT_Timer2)
	COMx_InitStructure.UART_BaudRate = 115200ul;   // 波特率,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable = ENABLE;	   // 接收允许,   ENABLE或DISABLE
	//	UART_Configuration(UART1, &COMx_InitStructure);		//初始化串口 UART1,UART2,UART3,UART4
	//	NVIC_UART1_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	UART_Configuration(UART2, &COMx_InitStructure); // 初始化串口 UART1,UART2,UART3,UART4
	NVIC_UART2_Init(ENABLE, Priority_0);			// 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART3, &COMx_InitStructure);		//初始化串口 UART1,UART2,UART3,UART4
	//	NVIC_UART3_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART4, &COMx_InitStructure);		//初始化串口 UART1,UART2,UART3,UART4
	//	NVIC_UART4_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

	//	UART1_SW(UART1_SW_P30_P31);		//UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
								//	UART3_SW(UART3_SW_P00_P01);		//UART3_SW_P00_P01,UART3_SW_P50_P51
								//	UART4_SW(UART4_SW_P02_P03);		//UART4_SW_P02_P03,UART4_SW_P52_P53
}
/****************  串口初始化函数 *****************/
void bsp_uart_init(void)
{
	DMA_UART_config();
	UART_config();
	GPIO_config();
}
//DMA_UR2T_TRIG();