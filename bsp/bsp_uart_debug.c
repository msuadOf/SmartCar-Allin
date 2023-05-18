#include "bsp.h"

#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_UART.h"
#include "STC32G_NVIC.h"
#include "STC32G_Delay.h"
#include "STC32G_Switch.h"

/*************	����˵��	**************

˫����ȫ˫���жϷ�ʽ�շ�ͨѶ����

ͨ��PC��MCU��������, MCU�յ���ͨ�����ڰ��յ�������ԭ������, Ĭ�ϲ����ʣ�115200,N,8,1.

ͨ������ UART.h ͷ�ļ������ UART1~UART4 ���壬������ͬͨ���Ĵ���ͨ�š�

�ö�ʱ���������ʷ�����������ʹ��1Tģʽ(���ǵͲ�������12T)����ѡ��ɱ�������������ʱ��Ƶ�ʣ�����߾��ȡ�

����ʱ, ѡ��ʱ�� 22.1184MHz (�û�����"config.h"�޸�Ƶ��).

******************************************/

/*************	���س�������	**************/

/*************	���ر�������	**************/

/*************	���غ�������	**************/

/*************  �ⲿ�����ͱ������� *****************/

/******************* IO���ú��� *******************/
static void GPIO_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; // �ṹ����

    GPIO_InitStructure.Pin = GPIO_Pin_6 | GPIO_Pin_7; // ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7
    GPIO_InitStructure.Mode = GPIO_PullUp;            // ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
    GPIO_Inilize(GPIO_P3, &GPIO_InitStructure);       // ��ʼ��
}

/***************  ���ڳ�ʼ������ *****************/
static void UART_config(void)
{
    COMx_InitDefine COMx_InitStructure; // �ṹ����

    COMx_InitStructure.UART_Mode = UART_8bit_BRTx;  // ģʽ, UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
    COMx_InitStructure.UART_BRT_Use = BRT_Timer2;   // ѡ�����ʷ�����, BRT_Timer1, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2)
    COMx_InitStructure.UART_BaudRate = 115200ul;    // ������, һ�� 110 ~ 115200
    COMx_InitStructure.UART_RxEnable = ENABLE;      // ��������,   ENABLE��DISABLE
    COMx_InitStructure.BaudRateDouble = DISABLE;    // �����ʼӱ�, ENABLE��DISABLE
    UART_Configuration(UART1, &COMx_InitStructure); // ��ʼ������1 UART1,UART2,UART3,UART4
    NVIC_UART1_Init(ENABLE, Priority_1);            // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

    UART1_SW(UART1_SW_P36_P37); // UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
}
void bsp_uart_debug(){
    GPIO_config();
    UART_config();

}