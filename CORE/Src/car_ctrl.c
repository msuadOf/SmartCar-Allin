#include "car_ctrl.h"
#include "bsp.h"
#include "MPU6500.h"
#include "math.h"

// #define SeekFree_Way
#define Less_PID_Way

// 初始化
void car_ctrl_init()
{
	bsp_pwm_init();		// 两路电机驱动pwm
	bsp_encoder_init(); // 编码器初始化

	MPU6500_Init();
	bsp_adc_dma_timer_init();
}

#ifdef SeekFree_Way

// 速度参数
/**
 * 需要调定的参数：
 *
 *
 */
int16 get_speed_l, get_speed_r, goal_speed_l = 0, goal_speed_r = 0, s_goal_speed = 0;
int err_speed_l[3] = {0}, err_speed_r[3] = {0};
float speed_pid_p = 10, speed_pid_i = 3, speed_pid_d = 10, speed_pid_out_l, speed_pid_out_r;
int16 up_speed = 0;

// 角速度参数
int16 get_angvel, goal_angvel = 0; // goal_angvel向左为负向右为正
float angvel_err, last_angvel_err;
float angvel_pid_p = 0.05, angvel_pid_i = 0, angvel_pid_d = 0.01, angvel_pid_out;

// 转向参数
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

// 设置左右轮目标速度
void set_goal_speed(int16 l, int16 r)
{
	goal_speed_l = l;
	goal_speed_r = r;
}

#define limit(x, y) ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
// 速度PID计算
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

// 设置目标角速度
void set_goal_angvel(int16 angvel)
{
	goal_angvel = angvel;
}

// 角速度PID计算
void angvel_pid_ctrl()
{
	MPU6500_get_gyro_buffer(gyro_buffer);
	get_angvel = gyro_buffer[2];
	angvel_err = goal_angvel - get_angvel;
	angvel_pid_out = angvel_err * angvel_pid_p + (angvel_err - last_angvel_err) * angvel_pid_d;
	last_angvel_err = angvel_err;
	set_goal_speed(s_goal_speed + angvel_pid_out, s_goal_speed - angvel_pid_out);
}

// 转向PID计算

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

	Sensor_Left = ADC_Value[6];	  // 左边电感采集值
	Sensor_Middle = ADC_Value[3]; // 中间电感采集值
	Sensor_Right = ADC_Value[0];  // 右边电感采集值

	Sensor_Left_M = ADC_Value[0];  // 左边电感采集值
	Sensor_Right_M = ADC_Value[5]; // 右边电感采集值

	// if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	// {
	// 	sum = sqrt(Sensor_Left) * 1 + Sensor_Middle * 50 + sqrt(Sensor_Right) * 99; // 归一化处理
	// 	Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);	// 求偏差
	// }
	// //Velocity = 35;									// 电磁巡线模式下的速度
	// Bias = Sensor - 50;								// 提取偏差
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
	Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-100.0f); // 方向PID*/ 外侧双电感
	// Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-80.0f); // 方向PID*/ 内侧双电感
	//  Angle = abs(Bias)*Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
	Last_Bias = Bias; // 上一次的偏差

	set_goal_angvel(turn_pid_out);
}

// 串级控制
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
 *    上面这个用编码器把角速度给闭了
 *    但是我们想用角速度传感器直接控制
 * 	  这是第二种
 *
 * ======================================
 */

#ifdef Less_PID_Way

// 速度参数
int16 get_speed_l, get_speed_r, goal_speed_l = 0, goal_speed_r = 0, s_goal_speed = 0;
int err_speed[3] = {0};
float speed_pid_p = 10, speed_pid_i = 3, speed_pid_d = 10, speed_pid_out, speed_pid_out_r;
int16 up_speed = 0;
float speed_pid_ctrl_current;

// 角速度参数
int16 get_angvel, goal_angvel = 0;
float angvel_err, last_angvel_err;
float angvel_pid_p = 0.05, angvel_pid_i = 0, angvel_pid_d = 0.01, angvel_pid_out;

// 转向参数
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
// 线速度pid
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
// 角速度pid
float angvel_pid_ctrl(float aim)
{
	MPU6500_get_gyro_buffer(gyro_buffer);
	get_angvel = gyro_buffer[2];
	angvel_err = aim - get_angvel;
	angvel_pid_out = angvel_err * angvel_pid_p + (angvel_err - last_angvel_err) * angvel_pid_d;
	last_angvel_err = angvel_err;
	return angvel_pid_out;
}
// 内环速度和角速度pid
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

	Sensor_Left = ADC_Value[6];	  // 左边电感采集值
	Sensor_Middle = ADC_Value[3]; // 中间电感采集值
	Sensor_Right = ADC_Value[0];  // 右边电感采集值

	Sensor_Left_M = ADC_Value[0];  // 左边电感采集值
	Sensor_Right_M = ADC_Value[5]; // 右边电感采集值

	// if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	// {
	// 	sum = sqrt(Sensor_Left) * 1 + Sensor_Middle * 50 + sqrt(Sensor_Right) * 99; // 归一化处理
	// 	Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);	// 求偏差
	// }
	// //Velocity = 35;									// 电磁巡线模式下的速度
	// Bias = Sensor - 50;								// 提取偏差
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
	Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-100.0f); // 方向PID*/ 外侧双电感
	// Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-80.0f); // 方向PID*/ 内侧双电感
	//  Angle = abs(Bias)*Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
	Last_Bias = Bias; // 上一次的偏差

	set_Car_Ctrl(20, Angle, 10);
	return Angle;
}

void car_ctrl()
{
	turn_pid_ctrl();
}
#endif // Less_PID_Way