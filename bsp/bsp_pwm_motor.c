#include "bsp.h"

#include "STC32G_PWM.h"

// 捕获比较模式寄存器
const uint32 PWM_CCMR_ADDR[] = {0x7efec8, 0x7efec9, 0x7efeca, 0x7efecb,
								0x7efee8, 0x7efee9, 0x7efeea, 0x7efeeb};
// 捕获比较使能寄存器
const uint32 PWM_CCER_ADDR[] = {0x7efecc, 0x7efecd,
								0x7efeec, 0x7efeed};
// 控制寄存器,高8位地址  低8位地址 + 1即可
const uint32 PWM_CCR_ADDR[] = {0x7efed5, 0x7efed7, 0x7efed9, 0x7efedb,
							   0x7efef5, 0x7efef7, 0x7efef9, 0x7efefb};

// 控制寄存器,高8位地址  低8位地址 + 1即可
const uint32 PWM_ARR_ADDR[] = {0x7efed2, 0x7efef2};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM_gpio初始化（内部使用用户无需关心）
//  @param      pwmch       PWM通道号及引脚
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_set_gpio(PWMCH_enum pwmch)
{
	switch (pwmch)
	{
	case PWMA_CH1P_P10:
	{
		gpio_mode(P1_0, GPO_PP);
		break;
	}
	case PWMA_CH1N_P11:
	{
		gpio_mode(P1_1, GPO_PP);
		break;
	}
	case PWMA_CH1P_P20:
	{
		gpio_mode(P2_0, GPO_PP);
		break;
	}
	case PWMA_CH1N_P21:
	{
		gpio_mode(P2_1, GPO_PP);
		break;
	}
	case PWMA_CH1P_P60:
	{
		gpio_mode(P6_0, GPO_PP);
		break;
	}
	case PWMA_CH1N_P61:
	{
		gpio_mode(P6_1, GPO_PP);
		break;
	}

	case PWMA_CH2P_P12:
	{
		gpio_mode(P1_2, GPO_PP);
		break;
	}
	case PWMA_CH2N_P13:
	{
		gpio_mode(P1_3, GPO_PP);
		break;
	}
	case PWMA_CH2P_P22:
	{
		gpio_mode(P2_2, GPO_PP);
		break;
	}
	case PWMA_CH2N_P23:
	{
		gpio_mode(P2_3, GPO_PP);
		break;
	}
	case PWMA_CH2P_P62:
	{
		gpio_mode(P6_2, GPO_PP);
		break;
	}
	case PWMA_CH2N_P63:
	{
		gpio_mode(P6_3, GPO_PP);
		break;
	}

	case PWMA_CH3P_P14:
	{
		gpio_mode(P1_4, GPO_PP);
		break;
	}
	case PWMA_CH3N_P15:
	{
		gpio_mode(P1_5, GPO_PP);
		break;
	}
	case PWMA_CH3P_P24:
	{
		gpio_mode(P2_4, GPO_PP);
		break;
	}
	case PWMA_CH3N_P25:
	{
		gpio_mode(P2_5, GPO_PP);
		break;
	}
	case PWMA_CH3P_P64:
	{
		gpio_mode(P6_4, GPO_PP);
		break;
	}
	case PWMA_CH3N_P65:
	{
		gpio_mode(P6_5, GPO_PP);
		break;
	}

	case PWMA_CH4P_P16:
	{
		gpio_mode(P1_6, GPO_PP);
		break;
	}
	case PWMA_CH4N_P17:
	{
		gpio_mode(P1_7, GPO_PP);
		break;
	}
	case PWMA_CH4P_P26:
	{
		gpio_mode(P2_6, GPO_PP);
		break;
	}
	case PWMA_CH4N_P27:
	{
		gpio_mode(P2_7, GPO_PP);
		break;
	}
	case PWMA_CH4P_P66:
	{
		gpio_mode(P6_6, GPO_PP);
		break;
	}
	case PWMA_CH4N_P67:
	{
		gpio_mode(P6_7, GPO_PP);
		break;
	}
	case PWMA_CH4P_P34:
	{
		gpio_mode(P3_4, GPO_PP);
		break;
	}
	case PWMA_CH4N_P33:
	{
		gpio_mode(P3_3, GPO_PP);
		break;
	}

	case PWMB_CH1_P20:
	{
		gpio_mode(P2_0, GPO_PP);
		break;
	}
	case PWMB_CH1_P17:
	{
		gpio_mode(P1_7, GPO_PP);
		break;
	}
	case PWMB_CH1_P00:
	{
		gpio_mode(P0_0, GPO_PP);
		break;
	}
	case PWMB_CH1_P74:
	{
		gpio_mode(P7_4, GPO_PP);
		break;
	}

	case PWMB_CH2_P21:
	{
		gpio_mode(P2_1, GPO_PP);
		break;
	}
	case PWMB_CH2_P54:
	{
		gpio_mode(P5_4, GPO_PP);
		break;
	}
	case PWMB_CH2_P01:
	{
		gpio_mode(P0_1, GPO_PP);
		break;
	}
	case PWMB_CH2_P75:
	{
		gpio_mode(P7_5, GPO_PP);
		break;
	}

	case PWMB_CH3_P22:
	{
		gpio_mode(P2_2, GPO_PP);
		break;
	}
	case PWMB_CH3_P33:
	{
		gpio_mode(P3_3, GPO_PP);
		break;
	}
	case PWMB_CH3_P02:
	{
		gpio_mode(P0_2, GPO_PP);
		break;
	}
	case PWMB_CH3_P76:
	{
		gpio_mode(P7_6, GPO_PP);
		break;
	}

	case PWMB_CH4_P23:
	{
		gpio_mode(P2_3, GPO_PP);
		break;
	}
	case PWMB_CH4_P34:
	{
		gpio_mode(P3_4, GPO_PP);
		break;
	}
	case PWMB_CH4_P03:
	{
		gpio_mode(P0_3, GPO_PP);
		break;
	}
	case PWMB_CH4_P77:
	{
		gpio_mode(P7_7, GPO_PP);
		break;
	}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM初始化
//  @param      pwmch       PWM通道号及引脚
//  @param      freq        PWM频率(10Hz-3MHz)
//  @param      duty        PWM占空比
//  @return     void
//  Sample usage:
//							pwm_init(PWM0_P00, 100, 5000);     //初始化PWM0  使用引脚P0.0  输出PWM频率100HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
//							PWM_DUTY_MAX在zf_pwm.h文件中 默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_init(PWMCH_enum pwmch, uint32 freq, float duty)
{

	uint32 match_temp;
	uint32 period_temp;
	uint16 freq_div = 0;

	P_SW2 |= 0x80;

	// GPIO需要设置为推挽输出
	pwm_set_gpio(pwmch);

	// 分频计算，周期计算，占空比计算
	freq_div = (MAIN_Fosc / freq) >> 16; // 多少分频
	period_temp = MAIN_Fosc / freq;
	period_temp = period_temp / (freq_div + 1) - 1; // 周期

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = period_temp * ((float)duty / PWM_DUTY_MAX); // 占空比
	}
	else
	{
		match_temp = period_temp + 1; // duty为100%
	}

	if (PWMB_CH1_P20 <= pwmch) // PWM5-8
	{
		// 通道选择，引脚选择
		PWMB_ENO |= (1 << ((2 * ((pwmch >> 4) - 4))));			   // 使能通道
		PWMB_PS |= ((pwmch & 0x03) << ((2 * ((pwmch >> 4) - 4)))); // 输出脚选择

		// 配置通道输出使能和极性
		(*(unsigned char volatile far *)(PWM_CCER_ADDR[pwmch >> 5])) |= (uint8)(1 << (((pwmch >> 4) & 0x01) * 4));

		// 设置预分频
		PWMB_PSCRH = (uint8)(freq_div >> 8);
		PWMB_PSCRL = (uint8)freq_div;

		PWMB_BRK = 0x80; // 主输出使能 相当于总开关
		PWMB_CR1 = 0x01; // PWM开始计数
	}
	else
	{
		PWMA_ENO |= (1 << (pwmch & 0x01)) << ((pwmch >> 4) * 2); // 使能通道
		PWMA_PS |= ((pwmch & 0x07) >> 1) << ((pwmch >> 4) * 2);	 // 输出脚选择

		// 配置通道输出使能和极性
		(*(unsigned char volatile far *)(PWM_CCER_ADDR[pwmch >> 5])) |= (1 << ((pwmch & 0x01) * 2 + ((pwmch >> 4) & 0x01) * 0x04));

		// 设置预分频
		PWMA_PSCRH = (uint8)(freq_div >> 8);
		PWMA_PSCRL = (uint8)freq_div;

		PWMA_BRK = 0x80; // 主输出使能 相当于总开关
		PWMA_CR1 = 0x01; // PWM开始计数
	}

	// 周期
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) = (uint8)(period_temp >> 8); // 高8位
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1)) = (uint8)period_temp;	 // 低8位

	// 设置捕获值|比较值
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // 高8位
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // 低8位

	// 功能设置
	(*(unsigned char volatile far *)(PWM_CCMR_ADDR[pwmch >> 4])) |= 0x06 << 4; // 设置为PWM模式1
	(*(unsigned char volatile far *)(PWM_CCMR_ADDR[pwmch >> 4])) |= 1 << 3;	   // 开启PWM寄存器的预装载功

	//	P_SW2 &= 0x7F;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM占空比设置
//  @param      pwmch       PWM通道号及引脚
//  @param      duty        PWM占空比
//  @return     void
//  Sample usage:           pwm_duty(PWM0_P00, 5000);     //初始化PWM0  使用引脚P0.0  输出PWM频率50HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
//							PWM_DUTY_MAX在fsl_pwm.h文件中 默认为100
//-------------------------------------------------------------------------------------------------------------------
void pwm_duty(PWMCH_enum pwmch, float duty)
{
	uint32 match_temp;
	uint32 arr = ((*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) << 8) | (*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1));

	//	P_SW2 |= 0x80;

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = arr * ((float)duty / PWM_DUTY_MAX); // 占空比
	}
	else
	{
		match_temp = arr + 1;
	}

	// 设置捕获值|比较值
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // 高8位
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // 低8位

	//	P_SW2 &= ~0x80;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM频率设置
//  @param      pwmch       PWM通道号及引脚
//  @param      freq        PWM频率(10Hz-3MHz)
//  @param      duty        PWM占空比
//  @return     void
//  Sample usage:           pwm_freq(PWM0_P00, 50, 5000);     //修改化PWM0  使用引脚P0.0  输出PWM频率50HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
//-------------------------------------------------------------------------------------------------------------------
void pwm_freq(PWMCH_enum pwmch, uint32 freq, float duty)
{
	uint32 match_temp;
	uint32 period_temp;
	uint16 freq_div = 0;

	// 分频计算，周期计算，占空比计算
	freq_div = (MAIN_Fosc / freq) >> 16; // 多少分频
	period_temp = MAIN_Fosc / freq;
	period_temp = period_temp / (freq_div + 1) - 1; // 周期

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = period_temp * ((float)duty / PWM_DUTY_MAX); // 占空比
	}
	else
	{
		match_temp = period_temp + 1; // duty为100%
	}

	//	P_SW2 |= 0x80;

	if (PWMB_CH1_P20 <= pwmch) // PWM5-8
	{
		// 设置预分频
		PWMB_PSCRH = (uint8)(freq_div >> 8);
		PWMB_PSCRL = (uint8)freq_div;
	}
	else
	{
		// 设置预分频
		PWMA_PSCRH = (uint8)(freq_div >> 8);
		PWMA_PSCRL = (uint8)freq_div;
	}

	// 周期
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) = (uint8)(period_temp >> 8); // 高8位
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1)) = (uint8)period_temp;	 // 低8位

	// 设置捕获值|比较值
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // 高8位
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // 低8位

	//	P_SW2 &= ~0x80;
}

#define PWM1_1 0x00 // P:P1.0  N:P1.1
#define PWM1_2 0x01 // P:P2.0  N:P2.1
#define PWM1_3 0x02 // P:P6.0  N:P6.1

#define PWM2_1 0x00 // P:P1.2/P5.4  N:P1.3
#define PWM2_2 0x04 // P:P2.2  N:P2.3
#define PWM2_3 0x08 // P:P6.2  N:P6.3

#define PWM3_1 0x00 // P:P1.4  N:P1.5
#define PWM3_2 0x10 // P:P2.4  N:P2.5
#define PWM3_3 0x20 // P:P6.4  N:P6.5

#define PWM4_1 0x00 // P:P1.6  N:P1.7
#define PWM4_2 0x40 // P:P2.6  N:P2.7
#define PWM4_3 0x80 // P:P6.6  N:P6.7
#define PWM4_4 0xC0 // P:P3.4  N:P3.3
/***************  PWM初始化函数 *****************/

//===========电机控制方式=============
#define BSP_PWM_MOTOR_MODE1 // mode1为双极性控制，电机一边同时有两个pwm
//#define BSP_PWM_MOTOR_MODE2 // mode2为单极性控制，电机一边同时只有一个pwm

#define MOTORL_PWM1 PWMA_CH1P_P60
#define MOTORL_PWM2 PWMA_CH1N_P61
#define MOTORR_PWM1 PWMA_CH2P_P62
#define MOTORR_PWM2 PWMA_CH2N_P63

#ifdef BSP_PWM_MOTOR_MODE1
static void bsp_pwm_init_mode1(void)
{
	pwm_init(MOTORL_PWM1, 21000, 50);
	pwm_init(MOTORL_PWM2, 21000, 50);
	pwm_init(MOTORL_PWM2, 21000, 50);
	pwm_init(MOTORR_PWM2, 21000, 50);

	// 寄存器重写
	PWMA_DTR = 0x0C; // 设置死区时间//0.5us

	PWMA_ENO = 0x00;
	PWMA_ENO |= ENO1P; // 使能输出
	PWMA_ENO |= ENO1N; // 使能输出
	PWMA_ENO |= ENO2P; // 使能输出//
	PWMA_ENO |= ENO2N; // 使能输出//
	//    PWMA_ENO |= ENO3P; //使能输出
	//    PWMA_ENO |= ENO3N; //使能输出
	//    PWMA_ENO |= ENO4P; //使能输出
	//    PWMA_ENO |= ENO4N; //使能输出

	// 高级PWM输出脚会自动设置为推挽输出模式
	PWMA_PS = 0x00;	   // 高级 PWM 通道输出脚选择位
	PWMA_PS |= PWM1_3; // 选择 PWM1_3 通道
	PWMA_PS |= PWM2_3; // 选择 PWM2_3 通道//
	//    PWMA_PS |= PWM3_3; //选择 PWM3_3 通道
	//    PWMA_PS |= PWM4_3; //选择 PWM4_3 通道

	PWMA_BRK = 0x80; // 使能主输出
	//    PWMA_IER = 0x01; //使能中断
	PWMA_CR1 |= 0x81; // 使能ARR预装载，开始计时

	pwm_duty(MOTORL_PWM1, 50);
	pwm_duty(MOTORL_PWM2, 50);
}
#endif // BSP_PWM_MOTOR_MODE1
#ifdef BSP_PWM_MOTOR_MODE2
static void bsp_pwm_init_mode2(void)
{
}
#endif // BSP_PWM_MOTOR_MODE2

void bsp_pwm_init(void)
{
#ifdef BSP_PWM_MOTOR_MODE1
	bsp_pwm_init_mode1();
#endif // BSP_PWM_MOTOR_MODE1

#ifdef BSP_PWM_MOTOR_MODE2
	bsp_pwm_init_mode2();
#endif // BSP_PWM_MOTOR_MODE2
}
// 对左电机进行占空比的设置，并进行限幅
// 取值范围0~100，限幅为3~97
#ifdef BSP_PWM_MOTOR_MODE1
static void bsp_pwm_motor_duty_set_mode1(float duty_l, float duty_r)
{
	if (duty_l < 3)
	{
		duty_l = 3;
	}
	if (duty_l > 97)
	{
		duty_l = 97;
	}

	if (duty_r < 3)
	{
		duty_r = 3;
	}
	if (duty_r > 97)
	{
		duty_r = 97;
	}

	pwm_duty(MOTORL_PWM1, duty_l);
	pwm_duty(MOTORL_PWM2, duty_r);
}
#endif // BSP_PWM_MOTOR_MODE1
#ifdef BSP_PWM_MOTOR_MODE2

#define limit(x, y) ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
static void bsp_pwm_motor_duty_set_mode2(float duty_l, float duty_r)
{
	// 对占空比限幅
	duty_l = (int)limit((float)duty_l, 97);
	duty_r = (int)limit((float)duty_r, 97);

	if (duty_l >= 0) // 左侧正转
	{
		pwm_duty(MOTORL_PWM1, duty_l);
		pwm_duty(MOTORL_PWM2, 0);
	}
	else // 左侧反转
	{
		pwm_duty(MOTORL_PWM1, 0);
		pwm_duty(MOTORL_PWM2, -duty_l);
	}

	if (duty_r >= 0) // 右侧正转
	{
		pwm_duty(MOTORR_PWM1, duty_r);
		pwm_duty(MOTORR_PWM2, 0);
	}
	else // 右侧反转
	{
		pwm_duty(MOTORR_PWM1, 0);
		pwm_duty(MOTORR_PWM2, -duty_r);
	}
}
#endif // BSP_PWM_MOTOR_MODE2

//motor控制函数，给值范围-100 ~ 100，正负表示正反转
void bsp_motor_control(float duty_l, float duty_r)
{
#ifdef BSP_PWM_MOTOR_MODE1
	bsp_pwm_motor_duty_set_mode1(duty_l * 2 - 100, duty_r * 2 - 100);
#endif // BSP_PWM_MOTOR_MODE1
#ifdef BSP_PWM_MOTOR_MODE2
	bsp_pwm_motor_duty_set_mode2(duty_l, duty_r);
#endif // BSP_PWM_MOTOR_MODE2
}
