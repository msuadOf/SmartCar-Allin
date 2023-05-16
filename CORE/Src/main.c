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
	//P4_MODE_IO_PU(GPIO_Pin_All);			//P4.0����Ϊ׼˫���
	//P2_MODE_IO_PU(GPIO_Pin_All); // P2 ����Ϊ׼˫���
	//P4_MODE_IN_HIZ(GPIO_Pin_All);
}
/************************ ��ʱ������ ****************************/
void	Timer_config(void)
{
	 TIM_InitTypeDef		TIM_InitStructure;						//�ṹ����
	// TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 100000UL));		//��ֵ,
	// TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	// Timer_Inilize(Timer0,&TIM_InitStructure);					//��ʼ��Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer0_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	// TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//��ֵ,
	// TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	// Timer_Inilize(Timer1,&TIM_InitStructure);					//��ʼ��Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//��ֵ
	// TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	// Timer_Inilize(Timer2,&TIM_InitStructure);					//��ʼ��Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer2_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

	// TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	// TIM_InitStructure.TIM_ClkOut    = ENABLE;					//�Ƿ������������, ENABLE��DISABLE
	// TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//��ֵ
	// TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	// Timer_Inilize(Timer3,&TIM_InitStructure);					//��ʼ��Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
	// NVIC_Timer3_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//�Ƿ������������, ENABLE��DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (200*12)));		//��ֵ
	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	Timer_Inilize(Timer4,&TIM_InitStructure);					//��ʼ��Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer4_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�
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
	WTST = 0;  // ���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();  // ��չSFR(XFR)����ʹ��
	CKCON = 0; // ��߷���XRAM�ٶ�

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
		/* ��ȡ�����Ǻͼ��ٶȼ����� */
		//MPU6500_get_buffer(gyro_buffer, acc_buffer);

		/* ��ӡ���� */
		GPIO_TogglePin(P45);
		//putchar('A');
		printf("cnt:%d\n", cnt);

		delay_ms(250);
	}
}