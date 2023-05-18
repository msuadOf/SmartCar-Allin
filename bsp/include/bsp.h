#ifndef _BSP_H
#define _BSP_H

#include "config.h"

//========================================
// ||
// ||
// ||             MPU6500�õ�spi
// ||
// ||
//========================================
void bsp_spi_init();
void bsp_spi_read(unsigned char addr, unsigned char *buffer, int len);
void bsp_spi_write(unsigned char addr, unsigned char *buffer, int len);


//========================================
// ||
// ||
// ||             ====uart==== 
// ||     ����Ϊbsp_tim_counnter.c��غ궨��
// ||
// ||
//========================================
void bsp_uart_init(void);

//========================================
// ||
// ||
// ||    ===������������==== 
// ||    ����Ϊbsp_tim_encoder.c��غ궨��
// ||
// ||
//========================================
//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
    CTIM0_P34=0,    
	CTIM1_P35,    	
	CTIM2_P12,     	
	CTIM3_P04,
	CTIM4_P06,
}TIM_counter_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
    TIM_0,    
	TIM_1,    	
	TIM_2,     	
	TIM_3,
	TIM_4,
}TIMN_enum;


#define TIM2_CLEAR_FLAG		AUXINTIF &= ~0x01; 
#define TIM3_CLEAR_FLAG		AUXINTIF &= ~0x02; // ���жϱ�־
#define TIM4_CLEAR_FLAG		AUXINTIF &= ~0x04; // ���жϱ�־


void   bsp_timer_count_init(TIM_counter_enum tim_n);
void   bsp_timer_count_clean(TIM_counter_enum tim_n);
uint16 bsp_timer_count_read(TIM_counter_enum tim_n);

void pit_timer_ms(TIMN_enum tim_n, float time_ms);

//API
void bsp_encoder_init();
void bsp_get_encoder(int16* speed_l, int16* speed_r);
//========================================
// ||
// ||
// ||             GPIO���
// ||
// ||
//========================================

#define GPIO_TogglePin(Pin) (Pin=(Pin)?0:1)


typedef enum
{
	NOPULL = 0,
    PULLUP = 1,
}PULL_enum;


typedef enum
{
	P0_0 = 0x00, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, 
	P1_0 = 0x10, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, 
	P2_0 = 0x20, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7, 
	P3_0 = 0x30, P3_1, P3_2, P3_3, P3_4, P3_5, P3_6, P3_7, 
	P4_0 = 0x40, P4_1, P4_2, P4_3, P4_4, P4_5, P4_6, P4_7, 
	P5_0 = 0x50, P5_1, P5_2, P5_3, P5_4, P5_5, P5_6, P5_7,
	P6_0 = 0x60, P6_1, P6_2, P6_3, P6_4, P6_5, P6_6, P6_7,
	P7_0 = 0x70, P7_1, P7_2, P7_3, P7_4, P7_5, P7_6, P7_7,
	
}PIN_enum;

typedef enum
{
	//pnm1 pnm0
	GPIO = 0,			//׼˫���(������)
	GPO_PP = 1,			//�������
	GPI_IMPEDANCE = 2,	//��������
	GPI_OD = 3,			//��©���
}GPIOMODE_enum;

void  gpio_pull_set(PIN_enum pin, PULL_enum pull);
void  gpio_mode(PIN_enum pin, GPIOMODE_enum mode);

//========================================
// ||
// ||
// ||             PWM���
// ||
// ||
//========================================

#define PWM_DUTY_MAX 100

typedef enum
{
	//PWMA��PWMB�����鲻ͬ��PWM
	
	//������PWMAͨ����
	//ͬһ��PWM��ͬһʱ�̣�ֻ����ͬһ��PWM�����
	//����:PWMA_CH1P_P10 �� PWMA_CH1N_P11����һ�������
	PWMA_CH1P_P10 = 0x00,PWMA_CH1N_P11,
	PWMA_CH1P_P20,		 PWMA_CH1N_P21,
	PWMA_CH1P_P60,		 PWMA_CH1N_P61,

	PWMA_CH2P_P12 = 0x10,//���������� USB �ں˵�Դ��ѹ��
	PWMA_CH2N_P13,          
	PWMA_CH2P_P22,		 PWMA_CH2N_P23,
	PWMA_CH2P_P62,		 PWMA_CH2N_P63,

	PWMA_CH3P_P14 = 0x20,PWMA_CH3N_P15,
	PWMA_CH3P_P24,		 PWMA_CH3N_P25,
	PWMA_CH3P_P64,		 PWMA_CH3N_P65,

	PWMA_CH4P_P16 = 0x30,PWMA_CH4N_P17,
	PWMA_CH4P_P26,		 PWMA_CH4N_P27,
	PWMA_CH4P_P66,		 PWMA_CH4N_P67,
	PWMA_CH4P_P34,		 PWMA_CH4N_P33,
	
	//������PWMBͨ����
	//ͬһ��PWM��ͬһʱ�̣�ֻ����ͬһ��PWM�����
	//����:PWMB_CH1_P20 �� PWMB_CH1_P17 ����ͬʱ��� 
	//���ǲ�ͬ��ͨ������ͬһʱ�������
	//����:PWMB_CH1_P20 �� PWMB_CH2_P21����ͬʱ���
	PWMB_CH1_P20 = 0x40,
	PWMB_CH1_P17,
	PWMB_CH1_P00,
	PWMB_CH1_P74,

	PWMB_CH2_P21 = 0x50,
	PWMB_CH2_P54,		//������Ϊ��λ����
	PWMB_CH2_P01,
	PWMB_CH2_P75,

	PWMB_CH3_P22 = 0x60,
	PWMB_CH3_P33,
	PWMB_CH3_P02,
	PWMB_CH3_P76,

	PWMB_CH4_P23 = 0x70,
	PWMB_CH4_P34,
	PWMB_CH4_P03,
	PWMB_CH4_P77,

}PWMCH_enum;


void pwm_init(PWMCH_enum pwmch,uint32 freq, float duty);
void pwm_duty(PWMCH_enum pwmch, float duty);
void pwm_freq(PWMCH_enum pwmch, uint32 freq, float duty);

//API
void bsp_pwm_init(void);
void bsp_motor_control(float duty_l, float duty_r);

//========================================
// ||
// ||
// ||     TIM+ADC+DMA��ʱ�ɼ��ص�
// ||
// ||
//========================================
void bsp_adc_dma_timer_init();
//��Ҫ��TIM4�Ļص��е��ã����ڴ���ADC+DMA
void bsp_adc_dma_timer_callback_triger(void);
uint16 bsp_get_ADC_DMABuffer(int CHx);

//========================================
// ||
// ||
// ||     UART1��printf
// ||
// ||
//========================================
void bsp_uart_debug();
#endif // !_BSP_H