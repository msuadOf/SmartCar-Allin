#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_Delay.h"

#include "MPU6500.h"
#include "bsp.h"

void GPIO_config(void)
{
	// P4_MODE_IO_PU(GPIO_Pin_0);			//P4.0����Ϊ׼˫���
	P2_MODE_IO_PU(GPIO_Pin_All); // P2 ����Ϊ׼˫���
}
float gyro_buffer[3], acc_buffer[3];
void main(void)
{
	WTST = 0;  // ���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();  // ��չSFR(XFR)����ʹ��
	CKCON = 0; // ��߷���XRAM�ٶ�

	GPIO_config();
	P20 = 0; // ��ʵ���LED��Դ

	MPU6500_Init();
	bsp_uart_init();

	EA = 1;

	while (1)
	{
		u8 buf = 0x5a;

		//bsp_spi_write(0xA5, &buf, 1);
		/* ��ȡ�����Ǻͼ��ٶȼ����� */
		MPU6500_get_buffer(gyro_buffer, acc_buffer);

		/* ��ӡ���� */
		printf("Gyro:%.2f,%.2f,%.2f\n", gyro_buffer[0], gyro_buffer[1], gyro_buffer[2], acc_buffer[0], acc_buffer[1], acc_buffer[2]);

		delay_ms(250);
	}
}