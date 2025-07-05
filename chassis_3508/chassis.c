//
#include "main.h"
#include "hchassis.h"
#include "hDBUS.h"
#include "INS_task.h"
#include "math.h"
#include "can.h"

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define KV 2.0F

// PID信息
PID_typedef PID1;
PID_typedef PID2;
PID_typedef PID3;
motor_measure_t motor_chassis[3] = {0}; // 电机信息

// 初始化滤波
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// 发送电机数据
void chassis_can_cmd(int16_t motor1, int16_t motor2, int16_t motor3)
{
	CAN_TxHeaderTypeDef chasis_tx_message;
	uint8_t chassis_txdata[6] = {0};
	uint32_t send_mail_box;
	chasis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chasis_tx_message.IDE = CAN_ID_STD;
	chasis_tx_message.RTR = CAN_RTR_DATA;
	chasis_tx_message.DLC = 0x08;
	chassis_txdata[0] = motor1 >> 8;
	chassis_txdata[1] = motor1;
	chassis_txdata[2] = motor2 >> 8;
	chassis_txdata[3] = motor2;
	chassis_txdata[4] = motor3 >> 8;
	chassis_txdata[5] = motor3;

	HAL_CAN_AddTxMessage(&hcan1, &chasis_tx_message, chassis_txdata, &send_mail_box);
}

// 接收电机信息
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
	for (int i = 0; i < 2; i++)
	{
		(ptr)->last_ecd = (ptr)->ecd;
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
		(ptr)->temperate = (data)[6];
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	switch (rx_header.StdId)
	{
	case CAN_3508_M1_ID:
	case CAN_3508_M2_ID:
	case CAN_3508_M3_ID:
	{
		static uint8_t i = 0;
		// get motor id
		i = rx_header.StdId - CAN_3508_M1_ID;
		get_motor_measure(&motor_chassis[i], rx_data);
		break;
	}
	default:
	{
		break;
	}
	}
}

// 关闭电机断电
void shut_up(void)
{
	DBUS_decode_val.key = 0;
	DBUS_decode_val.mod = 0;
	for (uint8_t i = 0; i < 4; i++)
	{
		DBUS_decode_val.rocker[i] = 0;
	}
	for (uint8_t i = 0; i < 2; i++)
	{
		DBUS_decode_val.sw[i] = 0;
	}
	DBUS_decode_val.roll = 0;
	chassis_can_cmd(0, 0, 0);
	HAL_CAN_Stop(&hcan1);
	HAL_CAN_Stop(&hcan2);
}

// 清除数值
void val_clear(void)
{
	DBUS_decode_val.key = 0;
	DBUS_decode_val.mod = 0;
	for (uint8_t i = 0; i < 4; i++)
	{
		DBUS_decode_val.rocker[i] = 0;
	}
	for (uint8_t i = 0; i < 2; i++)
	{
		DBUS_decode_val.sw[i] = 0;
	}
	DBUS_decode_val.roll = 0;
}

// 初始化pid
void pid_init()
{
	// 初始化PID1
	PID1.kp = KP;
	PID1.ki = KI;
	PID1.kd = KD;
	PID1.out_max = OUT_MAX;
	PID1.iout_max = IOUT_MAX;
	PID1.pout = 0;
	PID1.iout = 0;
	PID1.dout = 0;
	PID1.out = 0;

	// 初始化PID2
	PID2.kp = KP;
	PID2.ki = KI;
	PID2.kd = KD;
	PID2.out_max = OUT_MAX;
	PID2.iout_max = IOUT_MAX;
	PID2.pout = 0;
	PID2.iout = 0;
	PID2.dout = 0;
	PID2.out = 0;

	// 初始化PID3
	PID3.kp = KP;
	PID3.ki = KI;
	PID3.kd = KD;
	PID3.out_max = OUT_MAX;
	PID3.iout_max = IOUT_MAX;
	PID3.pout = 0;
	PID3.iout = 0;
	PID3.dout = 0;
	PID3.out = 0;
}

double yaw;

// 控制pid电机控制程序
double chassis_motor_1_pid() // 电机1
{
	double vx = DBUS_decode_val.rocker[2] * KV;
	double vy = DBUS_decode_val.rocker[3] * KV;
	double vc = DBUS_decode_val.rocker[0] * KV;

	yaw = -get_INS_angle_point()[0];
	yaw = round(yaw * 100.0) / 100.0; // 四舍五入保留两位小数，减少噪声
	if (yaw >= -0.05 && yaw <= 0.05)
		yaw = 0; // 增大死区，减少抖动
	double sin_yaw = sin(yaw);
	double cos_yaw = cos(yaw);
	double vx_set = cos_yaw * vx - sin_yaw * vy;
	double vy_set = sin_yaw * vx + cos_yaw * vy;
	vx = vx_set;
	vy = vy_set;

	double target_val = (-vx + vc) * 6.0F;
	double current_val = motor_chassis[0].speed_rpm;

	PID1.his_error = PID1.cur_error;
	PID1.cur_error = target_val - current_val;
	PID1.pout = KP * PID1.cur_error;
	PID1.iout += KI * PID1.cur_error;
	PID1.dout = KD * (PID1.cur_error - PID1.his_error);
	PID1.iout = ((PID1.iout > IOUT_MAX) ? IOUT_MAX : PID1.iout);
	PID1.iout = ((PID1.iout < -IOUT_MAX) ? -IOUT_MAX : PID1.iout);
	if (target_val == 0 && current_val == 0 && PID1.cur_error == 0)
	{
		PID1.iout = 0;
	}
	// 添加输出死区，减少小幅振荡
	if (fabs(PID1.cur_error) < 10.0)
	{
		PID1.out = 0;
	}
	else
	{
		PID1.out = PID1.pout + PID1.iout + PID1.dout;
	}
	PID1.out = ((PID1.out > OUT_MAX) ? OUT_MAX : PID1.out);
	PID1.out = ((PID1.out < -OUT_MAX) ? -OUT_MAX : PID1.out);

	return PID1.out;
}

double chassis_motor_2_pid() // 电机2
{
	double vx = DBUS_decode_val.rocker[2];
	double vy = DBUS_decode_val.rocker[3] * KV;
	double vc = DBUS_decode_val.rocker[0] * KV;

	yaw = -get_INS_angle_point()[0];
	yaw = round(yaw * 100.0) / 100.0; // 四舍五入保留两位小数，减少噪声
	if (yaw >= -0.05 && yaw <= 0.05)
		yaw = 0; // 增大死区，减少抖动
	double sin_yaw = sin(yaw);
	double cos_yaw = cos(yaw);
	double vx_set = cos_yaw * vx - sin_yaw * vy;
	double vy_set = sin_yaw * vx + cos_yaw * vy;
	vx = vx_set;
	vy = vy_set;

	double target_val = (vx + vy + vc) * 6.0F;
	double current_val = motor_chassis[1].speed_rpm;

	PID2.his_error = PID2.cur_error;
	PID2.cur_error = target_val - current_val;
	PID2.pout = KP * PID2.cur_error;
	PID2.iout += KI * PID2.cur_error;
	PID2.dout = KD * (PID2.cur_error - PID2.his_error);
	PID2.iout = ((PID2.iout > IOUT_MAX) ? IOUT_MAX : PID2.iout);
	PID2.iout = ((PID2.iout < -IOUT_MAX) ? -IOUT_MAX : PID2.iout);
	if (target_val == 0 && current_val == 0 && PID2.cur_error == 0)
	{
		PID2.iout = 0;
	}
	// 添加输出死区，减少小幅振荡
	if (fabs(PID2.cur_error) < 10.0)
	{
		PID2.out = 0;
	}
	else
	{
		PID2.out = PID2.pout + PID2.iout + PID2.dout;
	}
	PID2.out = ((PID2.out > OUT_MAX) ? OUT_MAX : PID2.out);
	PID2.out = ((PID2.out < -OUT_MAX) ? -OUT_MAX : PID2.out);
	return PID2.out;
}

double chassis_motor_3_pid() // 电机3
{
	double vx = DBUS_decode_val.rocker[2];
	double vy = DBUS_decode_val.rocker[3] * KV;
	double vc = DBUS_decode_val.rocker[0] * KV;

	yaw = -get_INS_angle_point()[0];
	yaw = round(yaw * 100.0) / 100.0; // 四舍五入保留两位小数，减少噪声
	if (yaw >= -0.05 && yaw <= 0.05)
		yaw = 0; // 增大死区，减少抖动
	double sin_yaw = sin(yaw);
	double cos_yaw = cos(yaw);
	double vx_set = cos_yaw * vx - sin_yaw * vy;
	double vy_set = sin_yaw * vx + cos_yaw * vy;
	vx = vx_set;
	vy = vy_set;

	// 三轮布局运动学：电机3在右上，240度角
	double target_val = (vx - vy + vc) * 6.0F;
	double current_val = motor_chassis[2].speed_rpm;

	PID3.his_error = PID3.cur_error;
	PID3.cur_error = target_val - current_val;
	PID3.pout = KP * PID3.cur_error;
	PID3.iout += KI * PID3.cur_error;
	PID3.dout = KD * (PID3.cur_error - PID3.his_error);
	PID3.iout = ((PID3.iout > IOUT_MAX) ? IOUT_MAX : PID3.iout);
	PID3.iout = ((PID3.iout < -IOUT_MAX) ? -IOUT_MAX : PID3.iout);
	if (target_val == 0 && current_val == 0 && PID3.cur_error == 0)
	{
		PID3.iout = 0;
	}
	// 添加输出死区，减少小幅振荡
	if (fabs(PID3.cur_error) < 10.0)
	{
		PID3.out = 0;
	}
	else
	{
		PID3.out = PID3.pout + PID3.iout + PID3.dout;
	}
	PID3.out = ((PID3.out > OUT_MAX) ? OUT_MAX : PID3.out);
	PID3.out = ((PID3.out < -OUT_MAX) ? -OUT_MAX : PID3.out);
	return PID3.out;
}