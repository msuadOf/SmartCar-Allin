#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_Delay.h"

#include "MPU6500.h"
#include "bsp.h"

void GPIO_config(void)
{
	// P4_MODE_IO_PU(GPIO_Pin_0);			//P4.0设置为准双向口
	P2_MODE_IO_PU(GPIO_Pin_All); // P2 设置为准双向口
}
float gyro_buffer[3], acc_buffer[3];
void main(void)
{
	WTST = 0;  // 设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();  // 扩展SFR(XFR)访问使能
	CKCON = 0; // 提高访问XRAM速度

	GPIO_config();
	P20 = 0; // 打开实验板LED电源

	MPU6500_Init();
	bsp_uart_init();

	EA = 1;

	while (1)
	{
		u8 buf = 0x5a;

		//bsp_spi_write(0xA5, &buf, 1);
		/* 获取陀螺仪和加速度计数据 */
		MPU6500_get_buffer(gyro_buffer, acc_buffer);

		/* 打印数据 */
		printf("Gyro:%.2f,%.2f,%.2f\n", gyro_buffer[0], gyro_buffer[1], gyro_buffer[2], acc_buffer[0], acc_buffer[1], acc_buffer[2]);

		delay_ms(250);
	}
}