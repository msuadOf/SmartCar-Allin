#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_Delay.h"
#include	"STC32G_Timer.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"

#include "MPU6500.h"
#include "bsp.h"

void GPIO_config(void)
{
	//P4_MODE_IO_PU(GPIO_Pin_All);			//P4.0设置为准双向口
	//P2_MODE_IO_PU(GPIO_Pin_All); // P2 设置为准双向口
	//P4_MODE_IN_HIZ(GPIO_Pin_All);
}
/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	 TIM_InitTypeDef		TIM_InitStructure;						//结构定义
	// TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 100000UL));		//初值,
	// TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	// Timer_Inilize(Timer0,&TIM_InitStructure);					//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

	// TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//初值,
	// TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	// Timer_Inilize(Timer1,&TIM_InitStructure);					//初始化Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer1_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//初值
	// TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	// Timer_Inilize(Timer2,&TIM_InitStructure);					//初始化Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer2_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = ENABLE;					//是否输出高速脉冲, ENABLE或DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//初值
	// TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	// Timer_Inilize(Timer3,&TIM_InitStructure);					//初始化Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer3_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (200*12)));		//初值
	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer4,&TIM_InitStructure);					//初始化Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer4_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级
}
float gyro_buffer[3], acc_buffer[3];
int cnt=0;
void Timer_ISR_Callback(int Timx){
	if(Timx==Timer4){
		cnt=bsp_timer_count_read(CTIM3_P04);
		bsp_timer_count_clean(CTIM3_P04);
	}
}
void main(void)
{
	WTST = 0;  // 设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();  // 扩展SFR(XFR)访问使能
	CKCON = 0; // 提高访问XRAM速度

	GPIO_config();
	Timer_config();

	MPU6500_Init();
	bsp_uart_init();

	bsp_timer_count_init(CTIM3_P04);
	EA = 1;
    P45=0;
	while (1)
	{
		u8 buf = 0x5a;

		//bsp_spi_write(0xA5, &buf, 1);
		/* 获取陀螺仪和加速度计数据 */
		//MPU6500_get_buffer(gyro_buffer, acc_buffer);

		/* 打印数据 */
		GPIO_TogglePin(P45);
		//putchar('A');
		printf("cnt:%d\n", cnt);

		delay_ms(250);
	}
}