#include "bsp.h"

#include "STC32G_GPIO.h"
#include "STC32G_Switch.h"
#include "STC32G_NVIC.h"

#include "STC32G_ADC.h"
#include "STC32G_DMA.h"
#include "STC32G_Timer.h"

#define ADC_CH 16  /* 1~16, ADC转换通道数, 需同步修改转换通道 */
#define ADC_DATA 6 /* 6~n, 每个通道ADC转换数据总数, 2*转换次数+4, 需同步修改转换次数 */

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

    P1_MODE_IN_HIZ(GPIO_Pin_All); // P1.0~P1.7 设置为高阻输入
}

/******************** ADC 配置 ********************/
static void ADC_config(void)
{
    ADC_InitTypeDef ADC_InitStructure; // 结构定义

    ADC_InitStructure.ADC_SMPduty = 31;                    // ADC 模拟信号采样时间控制, 0~31（注意： SMPDUTY 一定不能设置小于 10）
    ADC_InitStructure.ADC_CsSetup = 0;                     // ADC 通道选择时间控制 0(默认),1
    ADC_InitStructure.ADC_CsHold = 1;                      // ADC 通道选择保持时间控制 0,1(默认),2,3
    ADC_InitStructure.ADC_Speed = ADC_SPEED_2X16T;         // 设置 ADC 工作时钟频率	ADC_SPEED_2X1T~ADC_SPEED_2X16T
    ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED; // ADC结果调整,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
    ADC_Inilize(&ADC_InitStructure);                       // 初始化
    ADC_PowerControl(ENABLE);                              // ADC电源开关, ENABLE或DISABLE
    NVIC_ADC_Init(DISABLE, Priority_0);                    // 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}
/******************** DMA 配置 ********************/
static void DMA_ADC_config(void)
{
    //***********DMA_ADC**************

    DMA_ADC_InitTypeDef DMA_ADC_InitStructure; // 结构定义

    DMA_ADC_InitStructure.DMA_Enable = ENABLE;           // DMA使能  	ENABLE,DISABLE
    DMA_ADC_InitStructure.DMA_Channel = 0xffff;          // ADC通道使能寄存器, 1:使能, bit15~bit0 对应 ADC15~ADC0
    DMA_ADC_InitStructure.DMA_Buffer = (u16)DmaAdBuffer; // ADC转换数据存储地址
    DMA_ADC_InitStructure.DMA_Times = ADC_4_Times;       // 每个通道转换次数, ADC_1_Times,ADC_2_Times,ADC_4_Times,ADC_8_Times,ADC_16_Times,ADC_32_Times,ADC_64_Times,ADC_128_Times,ADC_256_Times
    DMA_ADC_Inilize(&DMA_ADC_InitStructure);             // 初始化
    NVIC_DMA_ADC_Init(ENABLE, Priority_3, Priority_3);   // 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
    DMA_ADC_TRIG();                                      // 触发启动转换
}

/*************	功能说明	**************

程序演示5个定时器的使用, 均使用16位自动重装.

定时器0做16位自动重装, 中断频率为100000Hz，中断函数从P6.7取反输出50KHz方波信号.

定时器1做16位自动重装, 中断频率为10000Hz，中断函数从P6.6取反输出5KHz方波信号.

定时器2做16位自动重装, 中断频率为1000Hz，中断函数从P6.5取反输出500Hz方波信号.

定时器3做16位自动重装, 中断频率为100Hz，中断函数从P6.4取反输出50Hz方波信号.

定时器4做16位自动重装, 中断频率为50Hz，中断函数从P6.3取反输出25Hz方波信号.

下载时, 选择时钟 24MHz (可以在配置文件"config.h"中修改).

******************************************/
/************************ 定时器配置 ****************************/
static void Timer_config(void)
{
    TIM_InitTypeDef TIM_InitStructure; // 结构定义
    //	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 100000UL));		//初值,
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
    //	Timer_Inilize(Timer0,&TIM_InitStructure);					//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

    //	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//初值,
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
    //	Timer_Inilize(Timer1,&TIM_InitStructure);					//初始化Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer1_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//初值
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
    //	Timer_Inilize(Timer2,&TIM_InitStructure);					//初始化Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer2_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

    //	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    //	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//是否输出高速脉冲, ENABLE或DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//初值
    //	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
    //	Timer_Inilize(Timer3,&TIM_InitStructure);					//初始化Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
    //	NVIC_Timer3_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T; // 指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
    TIM_InitStructure.TIM_ClkOut = ENABLE;           // 是否输出高速脉冲, ENABLE或DISABLE
    //	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (1000*12)));		//初值
    TIM_InitStructure.TIM_Value = (u16)(65536UL - (MAIN_Fosc / (500 * 12)));
    TIM_InitStructure.TIM_Run = ENABLE;        // 是否初始化后启动定时器, ENABLE或DISABLE
    Timer_Inilize(Timer4, &TIM_InitStructure); // 初始化Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
    NVIC_Timer4_Init(ENABLE, NULL);            // 中断使能, ENABLE/DISABLE; 无优先级
}
void bsp_adc_dma_timer_init()
{
    ADC_config();
    DMA_ADC_config();
    Timer_config();
    GPIO_config();
}

//需要在TIM4的回调中调用，用于触发ADC+DMA
void bsp_adc_dma_timer_callback_triger(void){
    DMA_ADC_TRIG();
}

//获取DMABuffer的值，CHx为通道序列
uint16 bsp_get_ADC_DMABuffer(int CHx)
{
    return (uint16)(DmaAdBuffer[CHx][5].data16);
}