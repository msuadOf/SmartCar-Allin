#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_Delay.h"
#include "STC32G_Timer.h"
#include "STC32G_GPIO.h"
#include "STC32G_NVIC.h"

#include "MPU6500.h"
#include "bsp.h"

void GPIO_config(void)
{
	// P4_MODE_IO_PU(GPIO_Pin_All);			//P4.0设置为准双向口
	// P2_MODE_IO_PU(GPIO_Pin_All); // P2 设置为准双向口
	// P4_MODE_IN_HIZ(GPIO_Pin_All);
}


int cnt = 0;
void Timer_ISR_Callback(int Timx)
{
	if (Timx == Timer4)
	{
		//		cnt=bsp_timer_count_read(CTIM3_P04);
		//		bsp_timer_count_clean(CTIM3_P04);
	}
}
void main(void)
{
	WTST = 0;  // 设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();  // 扩展SFR(XFR)访问使能
	CKCON = 0; // 提高访问XRAM速度

	GPIO_config();


	MPU6500_Init();
	bsp_uart_init();
	//bsp_adc_dma_timer_init();
	bsp_pwm_init();
	bsp_spi_init();
	bsp_encoder_init();
	bsp_uart_debug();

	EA = 1;
	P45 = 0;
	delay_ms(100);
	while (1)
	{
		u8 buf = 0x5a;

		// bsp_spi_write(0xA5, &buf, 1);
		/* 获取陀螺仪和加速度计数据 */
		 //MPU6500_get_buffer(gyro_buffer, acc_buffer);

		/* 打印数据 */
		GPIO_TogglePin(P45);
		 //putchar('A');

		
		printf("%x\n",MPU6500_Read_u8(MPU6500_WHO_AM_I));
		//printf("gyro:%f,%f,%f\n", gyro_buffer[0],gyro_buffer[1],gyro_buffer[2]);

		delay_ms(500);
	}
}