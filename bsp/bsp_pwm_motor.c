#include "bsp.h"

#include "STC32G_PWM.h"

// ����Ƚ�ģʽ�Ĵ���
const uint32 PWM_CCMR_ADDR[] = {0x7efec8, 0x7efec9, 0x7efeca, 0x7efecb,
								0x7efee8, 0x7efee9, 0x7efeea, 0x7efeeb};
// ����Ƚ�ʹ�ܼĴ���
const uint32 PWM_CCER_ADDR[] = {0x7efecc, 0x7efecd,
								0x7efeec, 0x7efeed};
// ���ƼĴ���,��8λ��ַ  ��8λ��ַ + 1����
const uint32 PWM_CCR_ADDR[] = {0x7efed5, 0x7efed7, 0x7efed9, 0x7efedb,
							   0x7efef5, 0x7efef7, 0x7efef9, 0x7efefb};

// ���ƼĴ���,��8λ��ַ  ��8λ��ַ + 1����
const uint32 PWM_ARR_ADDR[] = {0x7efed2, 0x7efef2};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM_gpio��ʼ�����ڲ�ʹ���û�������ģ�
//  @param      pwmch       PWMͨ���ż�����
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
//  @brief      PWM��ʼ��
//  @param      pwmch       PWMͨ���ż�����
//  @param      freq        PWMƵ��(10Hz-3MHz)
//  @param      duty        PWMռ�ձ�
//  @return     void
//  Sample usage:
//							pwm_init(PWM0_P00, 100, 5000);     //��ʼ��PWM0  ʹ������P0.0  ���PWMƵ��100HZ   ռ�ձ�Ϊ�ٷ�֮ 5000/PWM_DUTY_MAX*100
//							PWM_DUTY_MAX��zf_pwm.h�ļ��� Ĭ��Ϊ10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_init(PWMCH_enum pwmch, uint32 freq, float duty)
{

	uint32 match_temp;
	uint32 period_temp;
	uint16 freq_div = 0;

	P_SW2 |= 0x80;

	// GPIO��Ҫ����Ϊ�������
	pwm_set_gpio(pwmch);

	// ��Ƶ���㣬���ڼ��㣬ռ�ձȼ���
	freq_div = (MAIN_Fosc / freq) >> 16; // ���ٷ�Ƶ
	period_temp = MAIN_Fosc / freq;
	period_temp = period_temp / (freq_div + 1) - 1; // ����

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = period_temp * ((float)duty / PWM_DUTY_MAX); // ռ�ձ�
	}
	else
	{
		match_temp = period_temp + 1; // dutyΪ100%
	}

	if (PWMB_CH1_P20 <= pwmch) // PWM5-8
	{
		// ͨ��ѡ������ѡ��
		PWMB_ENO |= (1 << ((2 * ((pwmch >> 4) - 4))));			   // ʹ��ͨ��
		PWMB_PS |= ((pwmch & 0x03) << ((2 * ((pwmch >> 4) - 4)))); // �����ѡ��

		// ����ͨ�����ʹ�ܺͼ���
		(*(unsigned char volatile far *)(PWM_CCER_ADDR[pwmch >> 5])) |= (uint8)(1 << (((pwmch >> 4) & 0x01) * 4));

		// ����Ԥ��Ƶ
		PWMB_PSCRH = (uint8)(freq_div >> 8);
		PWMB_PSCRL = (uint8)freq_div;

		PWMB_BRK = 0x80; // �����ʹ�� �൱���ܿ���
		PWMB_CR1 = 0x01; // PWM��ʼ����
	}
	else
	{
		PWMA_ENO |= (1 << (pwmch & 0x01)) << ((pwmch >> 4) * 2); // ʹ��ͨ��
		PWMA_PS |= ((pwmch & 0x07) >> 1) << ((pwmch >> 4) * 2);	 // �����ѡ��

		// ����ͨ�����ʹ�ܺͼ���
		(*(unsigned char volatile far *)(PWM_CCER_ADDR[pwmch >> 5])) |= (1 << ((pwmch & 0x01) * 2 + ((pwmch >> 4) & 0x01) * 0x04));

		// ����Ԥ��Ƶ
		PWMA_PSCRH = (uint8)(freq_div >> 8);
		PWMA_PSCRL = (uint8)freq_div;

		PWMA_BRK = 0x80; // �����ʹ�� �൱���ܿ���
		PWMA_CR1 = 0x01; // PWM��ʼ����
	}

	// ����
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) = (uint8)(period_temp >> 8); // ��8λ
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1)) = (uint8)period_temp;	 // ��8λ

	// ���ò���ֵ|�Ƚ�ֵ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // ��8λ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // ��8λ

	// ��������
	(*(unsigned char volatile far *)(PWM_CCMR_ADDR[pwmch >> 4])) |= 0x06 << 4; // ����ΪPWMģʽ1
	(*(unsigned char volatile far *)(PWM_CCMR_ADDR[pwmch >> 4])) |= 1 << 3;	   // ����PWM�Ĵ�����Ԥװ�ع�

	//	P_SW2 &= 0x7F;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMռ�ձ�����
//  @param      pwmch       PWMͨ���ż�����
//  @param      duty        PWMռ�ձ�
//  @return     void
//  Sample usage:           pwm_duty(PWM0_P00, 5000);     //��ʼ��PWM0  ʹ������P0.0  ���PWMƵ��50HZ   ռ�ձ�Ϊ�ٷ�֮ 5000/PWM_DUTY_MAX*100
//							PWM_DUTY_MAX��fsl_pwm.h�ļ��� Ĭ��Ϊ100
//-------------------------------------------------------------------------------------------------------------------
void pwm_duty(PWMCH_enum pwmch, float duty)
{
	uint32 match_temp;
	uint32 arr = ((*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) << 8) | (*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1));

	//	P_SW2 |= 0x80;

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = arr * ((float)duty / PWM_DUTY_MAX); // ռ�ձ�
	}
	else
	{
		match_temp = arr + 1;
	}

	// ���ò���ֵ|�Ƚ�ֵ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // ��8λ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // ��8λ

	//	P_SW2 &= ~0x80;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMƵ������
//  @param      pwmch       PWMͨ���ż�����
//  @param      freq        PWMƵ��(10Hz-3MHz)
//  @param      duty        PWMռ�ձ�
//  @return     void
//  Sample usage:           pwm_freq(PWM0_P00, 50, 5000);     //�޸Ļ�PWM0  ʹ������P0.0  ���PWMƵ��50HZ   ռ�ձ�Ϊ�ٷ�֮ 5000/PWM_DUTY_MAX*100
//-------------------------------------------------------------------------------------------------------------------
void pwm_freq(PWMCH_enum pwmch, uint32 freq, float duty)
{
	uint32 match_temp;
	uint32 period_temp;
	uint16 freq_div = 0;

	// ��Ƶ���㣬���ڼ��㣬ռ�ձȼ���
	freq_div = (MAIN_Fosc / freq) >> 16; // ���ٷ�Ƶ
	period_temp = MAIN_Fosc / freq;
	period_temp = period_temp / (freq_div + 1) - 1; // ����

	if (duty != PWM_DUTY_MAX)
	{
		match_temp = period_temp * ((float)duty / PWM_DUTY_MAX); // ռ�ձ�
	}
	else
	{
		match_temp = period_temp + 1; // dutyΪ100%
	}

	//	P_SW2 |= 0x80;

	if (PWMB_CH1_P20 <= pwmch) // PWM5-8
	{
		// ����Ԥ��Ƶ
		PWMB_PSCRH = (uint8)(freq_div >> 8);
		PWMB_PSCRL = (uint8)freq_div;
	}
	else
	{
		// ����Ԥ��Ƶ
		PWMA_PSCRH = (uint8)(freq_div >> 8);
		PWMA_PSCRL = (uint8)freq_div;
	}

	// ����
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6])) = (uint8)(period_temp >> 8); // ��8λ
	(*(unsigned char volatile far *)(PWM_ARR_ADDR[pwmch >> 6] + 1)) = (uint8)period_temp;	 // ��8λ

	// ���ò���ֵ|�Ƚ�ֵ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4])) = match_temp >> 8;		 // ��8λ
	(*(unsigned char volatile far *)(PWM_CCR_ADDR[pwmch >> 4] + 1)) = (uint8)match_temp; // ��8λ

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
/***************  PWM��ʼ������ *****************/
void bsp_pwm_init(void)
{
	pwm_init(PWMA_CH1P_P60, 21000, 50);
	pwm_init(PWMA_CH1N_P61, 21000, 50);
	pwm_init(PWMA_CH2P_P62, 21000, 50);
	pwm_init(PWMA_CH2N_P63, 21000, 50);

	// �Ĵ�����д
	PWMA_DTR = 0x0C; // ��������ʱ��//0.5us

	PWMA_ENO = 0x00;
	PWMA_ENO |= ENO1P; // ʹ�����
	PWMA_ENO |= ENO1N; // ʹ�����
	PWMA_ENO |= ENO2P; // ʹ�����//
	PWMA_ENO |= ENO2N; // ʹ�����//
	//    PWMA_ENO |= ENO3P; //ʹ�����
	//    PWMA_ENO |= ENO3N; //ʹ�����
	//    PWMA_ENO |= ENO4P; //ʹ�����
	//    PWMA_ENO |= ENO4N; //ʹ�����

	// �߼�PWM����Ż��Զ�����Ϊ�������ģʽ
	PWMA_PS = 0x00;	   // �߼� PWM ͨ�������ѡ��λ
	PWMA_PS |= PWM1_3; // ѡ�� PWM1_3 ͨ��
	PWMA_PS |= PWM2_3; // ѡ�� PWM2_3 ͨ��//
	//    PWMA_PS |= PWM3_3; //ѡ�� PWM3_3 ͨ��
	//    PWMA_PS |= PWM4_3; //ѡ�� PWM4_3 ͨ��

	PWMA_BRK = 0x80; // ʹ�������
	//    PWMA_IER = 0x01; //ʹ���ж�
	PWMA_CR1 |= 0x81; // ʹ��ARRԤװ�أ���ʼ��ʱ

	pwm_duty(PWMA_CH1P_P60, 50);
	pwm_duty(PWMA_CH2P_P62, 50);
}
// ����������ռ�ձȵ����ã��������޷�
// ȡֵ��Χ0~100���޷�Ϊ3~97
void bsp_pwm_motor_left_duty_set(float duty)
{
	if (duty < 3)
	{
		duty = 3;
	}
	if (duty > 97)
	{
		duty = 97;
	}
	pwm_duty(PWMA_CH1P_P60, duty);
}
void bsp_pwm_motor_right_duty_set(float duty)
{
	if (duty < 3)
	{
		duty = 3;
	}
	if (duty > 97)
	{
		duty = 97;
	}
	pwm_duty(PWMA_CH2P_P62, duty);
}