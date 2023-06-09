#include "bsp.h"
#include "config.h"
#include "stc32g.h"

/**
 * @brief 注意：
 * 			本文件所有函数copy自逐飞BLDC开源库，用来给编码器使用
 * 			例程查找逐飞库car_ctrl的文件相关
 *
 */

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器初始化作为外部计数
//  @param      tim_n           选择模块
//  @return     void
//  @since      v1.0
//  Sample usage:               bsp_timer_count_init(CTIM0_P34);		//初始化定时器0，外部输入为P3.4引脚
//  @note                       串口1使用定时器1作为波特率发生器，
//								串口2使用定时器2作为波特率发生器，
//								串口3使用定时器3作为波特率发生器，
//								串口4使用定时器4作为波特率发生器，
//                              STC16F仅有定时器0-定时器4，这5个定时器。
//								编码器采集数据也需要定时器作为外部计数。
//-------------------------------------------------------------------------------------------------------------------
void bsp_timer_count_init(TIM_counter_enum tim_n)
{

	switch (tim_n)
	{
	case CTIM0_P34:
	{
		TL0 = 0;
		TH0 = 0;
		TMOD |= 0x04; // 外部计数模式
		TR0 = 1;	  // 启动定时器
		break;
	}

	case CTIM1_P35:
	{
		TL1 = 0x00;
		TH1 = 0x00;
		TMOD |= 0x40; // 外部计数模式
		TR1 = 1;	  // 启动定时器
		break;
	}

	case CTIM2_P12:
	{
		T2L = 0x00;
		T2H = 0x00;
		AUXR |= 0x18; // 设置外部计数模式并启动定时器
		break;
	}

	case CTIM3_P04:
	{
		T3L = 0;
		T3H = 0;
		T4T3M |= 0x0c; // 设置外部计数模式并启动定时器
		break;
	}

	case CTIM4_P06:
	{
		T4L = 0;
		T4H = 0;
		T4T3M |= 0xc0; // 设置外部计数模式并启动定时器
		break;
	}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取计数数值
//  @param      countch     计数通道号及引脚
//  @return     uint32      返回计数值
//  Sample usage:           num = bsp_timer_count_read(CTIM0_P34);
//-------------------------------------------------------------------------------------------------------------------
uint16 bsp_timer_count_read(TIM_counter_enum tim_n)
{
	uint16 dat = 0;

	switch (tim_n)
	{
	case CTIM0_P34:
	{
		dat = (uint16)TH0 << 8;
		dat = ((uint8)TL0) | dat;
		break;
	}
	case CTIM1_P35:
	{
		dat = (uint16)TH1 << 8;
		dat = ((uint8)TL1) | dat;
		break;
	}
	case CTIM2_P12:
	{
		dat = (uint16)T2H << 8;
		dat = ((uint8)T2L) | dat;
		break;
	}
	case CTIM3_P04:
	{
		dat = (uint16)T3H << 8;
		dat = ((uint8)T3L) | dat;
		break;
	}
	case CTIM4_P06:
	{
		dat = (uint16)T4H << 8;
		dat = ((uint8)T4L) | dat;
		break;
	}
	}
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清除计数数值
//  @param      countch     计数通道号及引脚
//  @return     void
//  Sample usage:           bsp_timer_count_clean(CTIM0_P34);
//-------------------------------------------------------------------------------------------------------------------
void bsp_timer_count_clean(TIM_counter_enum tim_n)
{
	switch (tim_n)
	{
	case CTIM0_P34:
	{
		TR0 = 0;
		TH0 = 0;
		TL0 = 0;
		TR0 = 1;
		break;
	}
	case CTIM1_P35:
	{
		TR1 = 0;
		TH1 = 0;
		TL1 = 0;
		TR1 = 1;
		break;
	}
	case CTIM2_P12:
	{
		AUXR &= ~(1 << 4);
		T2H = 0;
		T2L = 0;
		AUXR |= 1 << 4;
		break;
	}
	case CTIM3_P04:
	{
		T4T3M &= ~(1 << 3);
		T3H = 0;
		T3L = 0;
		T4T3M |= (1 << 3);
		break;
	}
	case CTIM4_P06:
	{
		T4T3M &= ~(1 << 7);
		T4H = 0;
		T4L = 0;
		T4T3M |= (1 << 7);
		break;
	}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器周期中断
//  @param      tim_n      定时器通道号
//  @param      time_ms    时间(ms)
//  @return     void
//  Sample usage:          pit_timer_ms(TIM_0, 10)
//						   使用定时器0做周期中断，时间10ms一次。
//-------------------------------------------------------------------------------------------------------------------
void pit_timer_ms(TIMN_enum tim_n, float time_ms)
{
	uint16 temp;
	temp = (uint16)65536 - (uint16)((float)MAIN_Fosc / (12 * (1000 / time_ms)));

	if (TIM_0 == tim_n)
	{
		TMOD |= 0x00; // 模式 0
		TL0 = temp;
		TH0 = temp >> 8;
		TR0 = 1; // 启动定时器
		ET0 = 1; // 使能定时器中断
	}
	else if (TIM_1 == tim_n)
	{
		TMOD |= 0x00; // 模式 0
		TL1 = temp;
		TH1 = temp >> 8;
		TR1 = 1; // 启动定时器
		ET1 = 1; // 使能定时器中断
	}
	else if (TIM_2 == tim_n)
	{
		T2L = temp;
		T2H = temp >> 8;
		AUXR |= 0x10; // 启动定时器
		IE2 |= 0x04;  // 使能定时器中断
	}
	else if (TIM_3 == tim_n)
	{
		T3L = temp;
		T3H = temp >> 8;
		T4T3M |= 0x08; // 启动定时器
		IE2 |= 0x20;   // 使能定时器中断
	}
	else if (TIM_4 == tim_n)
	{
		T4L = temp;
		T4H = temp >> 8;
		T4T3M |= 0x80; // 启动定时器
		IE2 |= 0x40;   // 使能定时器中断
	}
}

#define ENCODER_L_A                 (CTIM0_P34)
#define ENCODER_L_B                 (P33)

#define ENCODER_R_A                 (CTIM1_P35)
#define ENCODER_R_B                 (P36)
void bsp_encoder_init()
{
	bsp_timer_count_init(ENCODER_L_A);
	bsp_timer_count_init(ENCODER_R_A);
}

void bsp_get_encoder(int16* speed_l, int16* speed_r)
{
	*speed_l = (int16)bsp_timer_count_read(ENCODER_L_A);
	*speed_r = (int16)bsp_timer_count_read(ENCODER_R_A);

	//计数器清零
	bsp_timer_count_clean(ENCODER_L_A);
	bsp_timer_count_clean(ENCODER_R_A);

	//采集方向信息
	if(0 == ENCODER_L_B)    
	{
		*speed_l = -*speed_l;
	}
	if(1 == ENCODER_R_B)    
	{
		*speed_r = -*speed_r;
	}
}
