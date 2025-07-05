//
#include "main.h"
#include "hchassis.h"
#include "hDBUS.h"
#include "INS_task.h"
#include "math.h"
#include "can.h"
#include "rob2.h"
#include <stdbool.h>

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define SPEED_SCALE 6.0F
#define KV 2.0F
#define KC 0.8F
#define KR 250.0F

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
	if(rx_header.IDE == CAN_ID_STD && hcan == &hcan1)
	{	
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
//	else if(rx_header.IDE == CAN_ID_EXT && hcan == &hcan2)
//	{
//		RobStrite_Motor_Analysis(&motor1,rx_data,rx_header.ExtId);
//		RobStrite_Motor_Analysis(&motor2,rx_data,rx_header.ExtId);
//		RobStrite_Motor_Analysis(&motor2,rx_data,rx_header.ExtId);
//	}
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

// 通用PID计算函数
static double calculate_pid(PID_typedef* pid, double target_val, double current_val, bool is_stop_mode)
{
    pid->his_error = pid->cur_error;
    pid->cur_error = target_val - current_val;
    pid->pout = KP * pid->cur_error;
    pid->iout += KI * pid->cur_error;
    pid->dout = KD * (pid->cur_error - pid->his_error);
    
    // 积分限幅
    pid->iout = ((pid->iout > IOUT_MAX) ? IOUT_MAX : pid->iout);
    pid->iout = ((pid->iout < -IOUT_MAX) ? -IOUT_MAX : pid->iout);
    
    // 积分清零条件
    if (is_stop_mode) {
        // 停止模式：当目标为0且电机速度很小时，清除积分项
        if (target_val == 0 && fabs(current_val) < 5.0) {
            pid->iout = 0;
        }
    } else {
        // 正常模式：当目标和当前都为0时，清除积分项
        if (target_val == 0 && current_val == 0 && pid->cur_error == 0) {
            pid->iout = 0;
        }
    }
    
    // 输出死区（仅在停止模式下）
    if (is_stop_mode && fabs(pid->cur_error) < 5.0) {
        pid->out = 0;
    } else {
        pid->out = pid->pout + pid->iout + pid->dout;
    }
    
    // 输出限幅
    pid->out = ((pid->out > OUT_MAX) ? OUT_MAX : pid->out);
    pid->out = ((pid->out < -OUT_MAX) ? -OUT_MAX : pid->out);
    
    return pid->out;
}

// 获取yaw角度并处理
static double get_processed_yaw(void)
{
    double processed_yaw = -get_INS_angle_point()[0];
    if (processed_yaw >= -0.1 && processed_yaw <= 0.1) {
        processed_yaw = 0;
    }
    return processed_yaw;
}

// 计算yaw角度修正值
static double calculate_yaw_correction(double vx, double vy, double vc)
{
    yaw = get_processed_yaw();
    double vr = yaw * KR;
    
    // 当有侧向运动或旋转时，禁用yaw修正
    if ((vx == 0 && vy == 0) || vy != 0 || vc != 0) {
        vr = 0;
    }
    
    return vr;
}

// 控制pid电机控制程序
double chassis_motor_1_pid() // 电机1
{
    double vx = DBUS_decode_val.rocker[2] * KV;
    double vy = DBUS_decode_val.rocker[3] * KV;
    double vc = DBUS_decode_val.rocker[0] * KC;
    
    double vr = calculate_yaw_correction(vx, vy, vc);
    double target_val = (-vx + vc + vr) * SPEED_SCALE;
    double current_val = motor_chassis[0].speed_rpm;
    
    return calculate_pid(&PID1, target_val, current_val, false);
}

double chassis_motor_2_pid() // 电机2
{
    double vx = DBUS_decode_val.rocker[2];
    double vy = DBUS_decode_val.rocker[3] * KV;
    double vc = DBUS_decode_val.rocker[0] * KC;
    
    // 电机2不使用yaw修正
    double target_val = (vx + vy + vc) * SPEED_SCALE;
    double current_val = motor_chassis[1].speed_rpm;
    
    return calculate_pid(&PID2, target_val, current_val, false);
}

double chassis_motor_3_pid() // 电机3
{
    double vx = DBUS_decode_val.rocker[2];
    double vy = DBUS_decode_val.rocker[3] * KV;
    double vc = DBUS_decode_val.rocker[0] * KC;
    
    // 电机3不使用yaw修正，三轮布局运动学：电机3在右上，240度角
    double target_val = (vx - vy + vc) * SPEED_SCALE;
    double current_val = motor_chassis[2].speed_rpm;
    
    return calculate_pid(&PID3, target_val, current_val, false);
}

// 停止模式的PID控制函数 - target_val=0但保持PID状态
double chassis_motor_1_pid_stop() // 电机1停止模式
{
    return calculate_pid(&PID1, 0.0F, motor_chassis[0].speed_rpm, true);
}

double chassis_motor_2_pid_stop() // 电机2停止模式
{
    return calculate_pid(&PID2, 0.0F, motor_chassis[1].speed_rpm, true);
}

double chassis_motor_3_pid_stop() // 电机3停止模式
{
    return calculate_pid(&PID3, 0.0F, motor_chassis[2].speed_rpm, true);
}