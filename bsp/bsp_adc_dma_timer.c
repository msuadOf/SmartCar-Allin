#include "bsp.h"

#include "STC32G_GPIO.h"
#include "STC32G_Switch.h"
#include "STC32G_NVIC.h"

#include "STC32G_ADC.h"
#include "STC32G_DMA.h"
#include "STC32G_Timer.h"

#define ADC_CH 16  /* 1~16, ADCת��ͨ����, ��ͬ���޸�ת��ͨ�� */
#define ADC_DATA 6 /* 6~n, ÿ��ͨ��ADCת����������, 2*ת������+4, ��ͬ���޸�ת������ */

typedef union
{
    u8 data8[2];
    u16 data16;
} ADC_DMA_Data_Union;

u8 chn = 0;
bit DmaADCFlag = 0;
ADC_DMA_Data_Union xdata DmaAdBuffer[ADC_CH][ADC_DATA];

static void GPIO_config(void)
{

    P1_MODE_IN_HIZ(GPIO_Pin_All); // P1.0~P1.7 ����Ϊ��������
}

/******************** ADC ���� ********************/
static void ADC_config(void)
{
    ADC_InitTypeDef ADC_InitStructure; // �ṹ����

    ADC_InitStructure.ADC_SMPduty = 31;                    // ADC ģ���źŲ���ʱ�����, 0~31��ע�⣺ SMPDUTY һ����������С�� 10��
    ADC_InitStructure.ADC_CsSetup = 0;                     // ADC ͨ��ѡ��ʱ����� 0(Ĭ��),1
    ADC_InitStructure.ADC_CsHold = 1;                      // ADC ͨ��ѡ�񱣳�ʱ����� 0,1(Ĭ��),2,3
    ADC_InitStructure.ADC_Speed = ADC_SPEED_2X16T;         // ���� ADC ����ʱ��Ƶ��	ADC_SPEED_2X1T~ADC_SPEED_2X16T
    ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED; // ADC�������,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
    ADC_Inilize(&ADC_InitStructure);                       // ��ʼ��
    ADC_PowerControl(ENABLE);                              // ADC��Դ����, ENABLE��DISABLE
    NVIC_ADC_Init(DISABLE, Priority_0);                    // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
}
/******************** DMA ���� ********************/
static void DMA_ADC_config(void)
{
    //***********DMA_ADC**************

    DMA_ADC_InitTypeDef DMA_ADC_InitStructure; // �ṹ����

    DMA_ADC_InitStructure.DMA_Enable = ENABLE;           // DMAʹ��  	ENABLE,DISABLE
    DMA_ADC_InitStructure.DMA_Channel = 0xffff;          // ADCͨ��ʹ�ܼĴ���, 1:ʹ��, bit15~bit0 ��Ӧ ADC15~ADC0
    DMA_ADC_InitStructure.DMA_Buffer = (u16)DmaAdBuffer; // ADCת�����ݴ洢��ַ
    DMA_ADC_InitStructure.DMA_Times = ADC_4_Times;       // ÿ��ͨ��ת������, ADC_1_Times,ADC_2_Times,ADC_4_Times,ADC_8_Times,ADC_16_Times,ADC_32_Times,ADC_64_Times,ADC_128_Times,ADC_256_Times
    DMA_ADC_Inilize(&DMA_ADC_InitStructure);             // ��ʼ��
    NVIC_DMA_ADC_Init(ENABLE, Priority_3, Priority_3);   // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
    DMA_ADC_TRIG();                                      // ��������ת��
}

/*************	����˵��	**************

������ʾ5����ʱ����ʹ��, ��ʹ��16λ�Զ���װ.

��ʱ��0��16λ�Զ���װ, �ж�Ƶ��Ϊ100000Hz���жϺ�����P6.7ȡ�����50KHz�����ź�.

��ʱ��1��16λ�Զ���װ, �ж�Ƶ��Ϊ10000Hz���жϺ�����P6.6ȡ�����5KHz�����ź�.

��ʱ��2��16λ�Զ���װ, �ж�Ƶ��Ϊ1000Hz���жϺ�����P6.5ȡ�����500Hz�����ź�.

��ʱ��3��16λ�Զ���װ, �ж�Ƶ��Ϊ100Hz���жϺ�����P6.4ȡ�����50Hz�����ź�.

��ʱ��4��16λ�Զ���װ, �ж�Ƶ��Ϊ50Hz���жϺ�����P6.3ȡ�����25Hz�����ź�.

����ʱ, ѡ��ʱ�� 24MHz (�����������ļ�"config.h"���޸�).

******************************************/
/************************ ��ʱ������ ****************************/
static void Timer_config(void)
{
    TIM_InitTypeDef TIM_InitStructure; // �ṹ����
    //	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 100000UL));		//��ֵ,
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
    //	Timer_Inilize(Timer0,&TIM_InitStructure);					//��ʼ��Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer0_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

    //	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//��ֵ,
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
    //	Timer_Inilize(Timer1,&TIM_InitStructure);					//��ʼ��Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//��ֵ
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
    //	Timer_Inilize(Timer2,&TIM_InitStructure);					//��ʼ��Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer2_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//�Ƿ������������, ENABLE��DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//��ֵ
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
    //	Timer_Inilize(Timer3,&TIM_InitStructure);					//��ʼ��Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer3_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T; // ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    TIM_InitStructure.TIM_ClkOut = ENABLE;           // �Ƿ������������, ENABLE��DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (1000*12)));		//��ֵ
    TIM_InitStructure.TIM_Value = (u16)(65536UL - (MAIN_Fosc / (500 * 12)));
    TIM_InitStructure.TIM_Run = ENABLE;        // �Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
    Timer_Inilize(Timer4, &TIM_InitStructure); // ��ʼ��Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
    NVIC_Timer4_Init(ENABLE, NULL);            // �ж�ʹ��, ENABLE/DISABLE; �����ȼ�
}
void bsp_adc_dma_timer_init()
{
    ADC_config();
    DMA_ADC_config();
    Timer_config();
    GPIO_config();
}

//��Ҫ��TIM4�Ļص��е��ã����ڴ���ADC+DMA
void bsp_adc_dma_timer_callback_triger(void){
    DMA_ADC_TRIG();
}

//��ȡDMABuffer��ֵ��CHxΪͨ������
uint16 bsp_get_ADC_DMABuffer(int CHx)
{
    return (uint16)(DmaAdBuffer[CHx][5].data16);
}