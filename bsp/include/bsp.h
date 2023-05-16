#ifndef _BSP_H
#define _BSP_H

#include "config.h"

void bsp_spi_init();
void bsp_spi_read(unsigned char addr, unsigned char *buffer, int len);
void bsp_spi_write(unsigned char addr, unsigned char *buffer, int len);

/**
 * @brief ====uart==== ����Ϊbsp_tim_counnter.c��غ궨��
 * 
 */
void bsp_uart_init(void);

/**
 * @brief ====������������==== ����Ϊbsp_tim_counnter.c��غ궨��
 * 
 */

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

//========================================
// ||
// ||
// ||             GPIO���
// ||
// ||
//========================================
typedef int GPIO_Pin_t ;
//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
	GPIOP0,
    GPIOP1,    
	GPIOP2,    	
	GPIOP3,     	
	GPIOP4,
	GPIOP5,
	GPIOP6
}GPIO_enum;
#define GPIO_TogglePin(Pin) (Pin=(Pin)?0:1)

#endif // !_BSP_H