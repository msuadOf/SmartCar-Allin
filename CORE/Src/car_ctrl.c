#include "car_ctrl.h"
#include "bsp.h"
#include "MPU6500.h"
#include "math.h"

// #define SeekFree_Way
#define Less_PID_Way

// ��ʼ��
void car_ctrl_init()
{
	bsp_pwm_init();		// ��·�������pwm
	bsp_encoder_init(); // ��������ʼ��

	MPU6500_Init();
	bsp_adc_dma_timer_init();
}

#ifdef SeekFree_Way

// �ٶȲ���
/**
 * ��Ҫ�����Ĳ�����
 *
 *
 */
int16 get_speed_l, get_speed_r, goal_speed_l = 0, goal_speed_r = 0, s_goal_speed = 0;
int err_speed_l[3] = {0}, err_speed_r[3] = {0};
float speed_pid_p = 10, speed_pid_i = 3, speed_pid_d = 10, speed_pid_out_l, speed_pid_out_r;
int16 up_speed = 0;

// ���ٶȲ���
int16 get_angvel, goal_angvel = 0; // goal_angvel����Ϊ������Ϊ��
float angvel_err, last_angvel_err;
float angvel_pid_p = 0.05, angvel_pid_i = 0, angvel_pid_d = 0.01, angvel_pid_out;

// ת�����
uint8 adc_get_l, adc_get_m, adc_get_r;
float turn_err, last_turn_err;
float turn_k1, turn_k2, turn_pid_dp, turn_pid_sp, turn_pid_dd, turn_pid_sd, turn_pid_out;

float Sensor_Left,
	Sensor_Middle,
	Sensor_Right,
	Sensor_Left_M,
	Sensor_Right_M,
	sum,
	Sensor, Bias, Last_Bias,
	Velocity, Angle,
	kA, kB, kC, kD;
float gyro_buffer[3], acc_buffer[3];
float roat_speed_action, roat_speed_aim, roat_speed_current, roat_speed_err, roat_speed_err_last = 0;
float ADC_Value[8];

// ����������Ŀ���ٶ�
void set_goal_speed(int16 l, int16 r)
{
	goal_speed_l = l;
	goal_speed_r = r;
}

#define limit(x, y) ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
// �ٶ�PID����
void speed_pid_ctrl()
{
	bsp_get_encoder(&get_speed_l, &get_speed_r);

	err_speed_l[2] = err_speed_l[1];
	err_speed_l[1] = err_speed_l[0];
	err_speed_l[0] = goal_speed_l - get_speed_l;

	speed_pid_out_l += (err_speed_l[0] - err_speed_l[1]) * speed_pid_p + (float)err_speed_l[0] * speed_pid_i + ((float)err_speed_l[0] - 2 * (float)err_speed_l[1] + (float)err_speed_l[2]) * speed_pid_d;

	speed_pid_out_l = limit(speed_pid_out_l, 100);

	err_speed_r[2] = err_speed_r[1];
	err_speed_r[1] = err_speed_r[0];
	err_speed_r[0] = goal_speed_r - get_speed_r;

	speed_pid_out_r += (err_speed_r[0] - err_speed_r[1]) * speed_pid_p + (float)err_speed_r[0] * speed_pid_i + ((float)err_speed_r[0] - 2 * (float)err_speed_r[1] + (float)err_speed_r[2]) * speed_pid_d;

	speed_pid_out_r = limit(speed_pid_out_r, 100);
	bsp_motor_control(speed_pid_out_l, speed_pid_out_r);
}

// ����Ŀ����ٶ�
void set_goal_angvel(int16 angvel)
{
	goal_angvel = angvel;
}

// ���ٶ�PID����
void angvel_pid_ctrl()
{
	MPU6500_get_gyro_buffer(gyro_buffer);
	get_angvel = gyro_buffer[2];
	angvel_err = goal_angvel - get_angvel;
	angvel_pid_out = angvel_err * angvel_pid_p + (angvel_err - last_angvel_err) * angvel_pid_d;
	last_angvel_err = angvel_err;
	set_goal_speed(s_goal_speed + angvel_pid_out, s_goal_speed - angvel_pid_out);
}

// ת��PID����

void turn_pid_ctrl()
{

	// ADC_Value_fill_in(&hLineHunting);
	ADC_Value[0] = bsp_get_ADC_DMABuffer(0);
	ADC_Value[1] = bsp_get_ADC_DMABuffer(1);
	ADC_Value[2] = bsp_get_ADC_DMABuffer(3);
	ADC_Value[3] = bsp_get_ADC_DMABuffer(4);
	ADC_Value[4] = bsp_get_ADC_DMABuffer(5);
	ADC_Value[5] = bsp_get_ADC_DMABuffer(6);
	ADC_Value[6] = bsp_get_ADC_DMABuffer(7);

	Sensor_Left = ADC_Value[6];	  // ��ߵ�вɼ�ֵ
	Sensor_Middle = ADC_Value[3]; // �м��вɼ�ֵ
	Sensor_Right = ADC_Value[0];  // �ұߵ�вɼ�ֵ

	Sensor_Left_M = ADC_Value[0];  // ��ߵ�вɼ�ֵ
	Sensor_Right_M = ADC_Value[5]; // �ұߵ�вɼ�ֵ

	// if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	// {
	// 	sum = sqrt(Sensor_Left) * 1 + Sensor_Middle * 50 + sqrt(Sensor_Right) * 99; // ��һ������
	// 	Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);	// ��ƫ��
	// }
	// //Velocity = 35;									// ���Ѳ��ģʽ�µ��ٶ�
	// Bias = Sensor - 50;								// ��ȡƫ��
	kB = 0.1;
	if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	{
		// Bias = 50.0 * ((1 * (Sensor_Left - Sensor_Right) +
		// 				kB * (Sensor_Left_M - Sensor_Right_M)) /
		// 			   (1 * (Sensor_Left + Sensor_Right) +
		// 				kB * (Sensor_Left_M + Sensor_Right_M)));

		Bias = 50.0 * (((Sensor_Left - Sensor_Right)) /
					   ((Sensor_Left + Sensor_Right)));

		if (fabs(Bias) < 0.5)
			Bias = 0;
	}
	else
	{
		Bias = 0;
	}
	Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-100.0f); // ����PID*/ ���˫���
	// Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-80.0f); // ����PID*/ �ڲ�˫���
	//  Angle = abs(Bias)*Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
	Last_Bias = Bias; // ��һ�ε�ƫ��

	set_goal_angvel(turn_pid_out);
}

// ��������
void car_ctrl()
{
	turn_pid_ctrl();
	angvel_pid_ctrl();
	speed_pid_ctrl();
}
#endif // SeekFree_Way

/**
 * =======================================
 *
 *    ��������ñ������ѽ��ٶȸ�����
 *    �����������ý��ٶȴ�����ֱ�ӿ���
 * 	  ���ǵڶ���
 *
 * ======================================
 */

#ifdef Less_PID_Way

// �ٶȲ���
int16 get_speed_l, get_speed_r, goal_speed_l = 0, goal_speed_r = 0, s_goal_speed = 0;
int err_speed[3] = {0};
float speed_pid_p = 10, speed_pid_i = 3, speed_pid_d = 10, speed_pid_out, speed_pid_out_r;
int16 up_speed = 0;
float speed_pid_ctrl_current;

// ���ٶȲ���
int16 get_angvel, goal_angvel = 0;
float angvel_err, last_angvel_err;
float angvel_pid_p = 0.05, angvel_pid_i = 0, angvel_pid_d = 0.01, angvel_pid_out;

// ת�����
uint8 adc_get_l, adc_get_m, adc_get_r;
float turn_err, last_turn_err;
float turn_k1, turn_k2, turn_pid_dp, turn_pid_sp, turn_pid_dd, turn_pid_sd, turn_pid_out;

float Sensor_Left,
	Sensor_Middle,
	Sensor_Right,
	Sensor_Left_M,
	Sensor_Right_M,
	sum,
	Sensor, Bias, Last_Bias,
	Velocity, Angle,
	kA, kB, kC, kD;
float gyro_buffer[3], acc_buffer[3];
float roat_speed_action, roat_speed_aim, roat_speed_current, roat_speed_err, roat_speed_err_last = 0;
float ADC_Value[8];

#define     limit(x, y)         ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
float get_Car_Speed_encoder()
{
	int16 r, l;
	float speed;
	bsp_get_encoder(&l, &r);
	speed = 0.5f * (l + r);
	return speed;
}
// ���ٶ�pid
float speed_pid_ctrl(float aim, float limit_val)
{
	speed_pid_ctrl_current = get_Car_Speed_encoder();
	err_speed[2] = err_speed[1];
	err_speed[1] = err_speed[0];
	err_speed[0] = aim - speed_pid_ctrl_current;

	speed_pid_out +=
		(err_speed[0] - err_speed[1]) * speed_pid_p +
		(float)err_speed[0] * speed_pid_i +
		((float)err_speed[0] - 2 * (float)err_speed[1] + (float)err_speed[2]) * speed_pid_d;

	speed_pid_out = limit(speed_pid_out, limit_val);
	return speed_pid_out;
}
// ���ٶ�pid
float angvel_pid_ctrl(float aim)
{
	MPU6500_get_gyro_buffer(gyro_buffer);
	get_angvel = gyro_buffer[2];
	angvel_err = aim - get_angvel;
	angvel_pid_out = angvel_err * angvel_pid_p + (angvel_err - last_angvel_err) * angvel_pid_d;
	last_angvel_err = angvel_err;
	return angvel_pid_out;
}
// �ڻ��ٶȺͽ��ٶ�pid
void set_Car_Ctrl(float speed_aim, float angle_aim, float speed_limit_val)
{
	float speed, angle;
	speed = speed_pid_ctrl(speed_aim, speed_limit_val);
	angle = angvel_pid_ctrl(angle_aim);
	bsp_motor_control(speed + angle, speed - angle);
}
float turn_pid_ctrl()
{
	// ADC_Value_fill_in(&hLineHunting);
	ADC_Value[0] = bsp_get_ADC_DMABuffer(0);
	ADC_Value[1] = bsp_get_ADC_DMABuffer(1);
	ADC_Value[2] = bsp_get_ADC_DMABuffer(3);
	ADC_Value[3] = bsp_get_ADC_DMABuffer(4);
	ADC_Value[4] = bsp_get_ADC_DMABuffer(5);
	ADC_Value[5] = bsp_get_ADC_DMABuffer(6);
	ADC_Value[6] = bsp_get_ADC_DMABuffer(7);

	Sensor_Left = ADC_Value[6];	  // ��ߵ�вɼ�ֵ
	Sensor_Middle = ADC_Value[3]; // �м��вɼ�ֵ
	Sensor_Right = ADC_Value[0];  // �ұߵ�вɼ�ֵ

	Sensor_Left_M = ADC_Value[0];  // ��ߵ�вɼ�ֵ
	Sensor_Right_M = ADC_Value[5]; // �ұߵ�вɼ�ֵ

	// if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	// {
	// 	sum = sqrt(Sensor_Left) * 1 + Sensor_Middle * 50 + sqrt(Sensor_Right) * 99; // ��һ������
	// 	Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);	// ��ƫ��
	// }
	// //Velocity = 35;									// ���Ѳ��ģʽ�µ��ٶ�
	// Bias = Sensor - 50;								// ��ȡƫ��
	kB = 0.1;
	if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	{
		// Bias = 50.0 * ((1 * (Sensor_Left - Sensor_Right) +
		// 				kB * (Sensor_Left_M - Sensor_Right_M)) /
		// 			   (1 * (Sensor_Left + Sensor_Right) +
		// 				kB * (Sensor_Left_M + Sensor_Right_M)));

		Bias = 50.0 * (((Sensor_Left - Sensor_Right)) /
					   ((Sensor_Left + Sensor_Right)));

		if (fabs(Bias) < 0.5)
			Bias = 0;
	}
	else
	{
		Bias = 0;
	}
	Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-100.0f); // ����PID*/ ���˫���
	// Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-80.0f); // ����PID*/ �ڲ�˫���
	//  Angle = abs(Bias)*Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
	Last_Bias = Bias; // ��һ�ε�ƫ��

	set_Car_Ctrl(20, Angle, 10);
	return Angle;
}

void car_ctrl()
{
	turn_pid_ctrl();
}
#endif // Less_PID_Way