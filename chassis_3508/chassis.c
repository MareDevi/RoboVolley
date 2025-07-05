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
#define SPEED_SCALE 7.8F
#define KR 290.0F


// 声明一个静态变量来保存目标航向角
static double target_yaw = 0.0;
static bool heading_lock_first_time = true;

// PID信息
PID_typedef PID1;
PID_typedef PID2;
PID_typedef PID3;
PID_typedef PID_yaw; // <-- 新增：为Yaw轴角度闭环准备的PID
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
	PID1.cur_error = 0;
	PID1.his_error = 0;

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
	PID2.cur_error = 0;
	PID2.his_error = 0;

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
	PID3.cur_error = 0;
	PID3.his_error = 0;

	// 初始化Yaw轴PID
	PID_yaw.kp = 50.0f;   // 降低Kp值，避免过度修正（原290.0f太大）
	PID_yaw.ki = 0.0f;    // Ki先设为0，防止积分饱和问题
	PID_yaw.kd = 5.0f;    // 添加小的微分项，提高稳定性
	PID_yaw.out_max = 100; // 进一步限制最大修正角速度（原1000太大）
	PID_yaw.iout_max = 50; // 降低积分限制（原500太大）
	PID_yaw.pout = 0;
	PID_yaw.iout = 0;
	PID_yaw.dout = 0;
	PID_yaw.out = 0;
	PID_yaw.cur_error = 0;
	PID_yaw.his_error = 0;
}

double yaw;

// 通用PID计算函数
static double calculate_pid(PID_typedef* pid, double target_val, double current_val, bool is_stop_mode)
{
    pid->his_error = pid->cur_error;
    pid->cur_error = target_val - current_val;
    
    // 使用结构体中的PID参数，而不是全局常量
    pid->pout = pid->kp * pid->cur_error;
    pid->iout += pid->ki * pid->cur_error;
    pid->dout = pid->kd * (pid->cur_error - pid->his_error);
    
    // 积分限幅 - 使用结构体中的限制值
    pid->iout = ((pid->iout > pid->iout_max) ? pid->iout_max : pid->iout);
    pid->iout = ((pid->iout < -pid->iout_max) ? -pid->iout_max : pid->iout);
    
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
    
    // 输出限幅 - 使用结构体中的限制值
    pid->out = ((pid->out > pid->out_max) ? pid->out_max : pid->out);
    pid->out = ((pid->out < -pid->out_max) ? -pid->out_max : pid->out);
    
    return pid->out;
}

double v1,v2,v3;  //调试用

/**
 * @brief 底盘主控制任务，包含航向锁定
 * @note  此函数应在一个循环中被周期性调用
 */
void chassis_control_task(void)
{
    // 1. 获取用户输入
    // 遥控器输入，映射到机器人坐标系速度
    double vx_in = DBUS_decode_val.rocker[2]; // 前后速度
    double vy_in = DBUS_decode_val.rocker[3]; // 左右速度
    double vz_in = DBUS_decode_val.rocker[0]; // 旋转速度

    // 2. 计算Yaw轴修正
    double current_yaw = -get_INS_angle_point()[0]; // 获取当前航向角，与之前保持一致
    double yaw_correction_speed = 0.0;

    // 航向锁定逻辑：当用户没有主动命令旋转时，启用航向锁定
    if (fabs(vz_in) < 10) // 设置一个摇杆死区，避免误触
    {
        // 如果是刚进入锁定模式，则将当前角度设为目标角度
        if (heading_lock_first_time)
        {
            target_yaw = current_yaw;
            heading_lock_first_time = false;
        }

        // 计算角度误差
        double yaw_error = target_yaw - current_yaw;
        
        // 添加角度死区，避免在静止时持续修正微小误差
        if (fabs(yaw_error) > 0.05) // 角度死区：约3度（假设单位是弧度）
        {
            // 使用通用PID计算函数来计算修正速度
            yaw_correction_speed = calculate_pid(&PID_yaw, target_yaw, current_yaw, false);
            
            // 限制修正速度的幅度，避免过大的修正
            if (yaw_correction_speed > 50) yaw_correction_speed = 50;
            if (yaw_correction_speed < -50) yaw_correction_speed = -50;
        }
        else
        {
            // 在死区内，不进行修正，并清除PID积分项
            yaw_correction_speed = 0.0;
            PID_yaw.iout = 0; // 清除积分项，防止累积
        }
    }
    else
    {
        // 如果用户正在主动旋转，则重置锁定状态，不进行修正
        heading_lock_first_time = true;
        yaw_correction_speed = 0.0;
        PID_yaw.iout = 0; // 清除积分项
    }

    // 3. 叠加角速度
    // 最终的机器人角速度 = 用户期望的角速度 + 航向修正角速度
    double vz_final = vz_in + yaw_correction_speed;

    // 4. 逆运动学解算
    // 使用 vx_in, vy_in, vz_final 计算三个轮子的目标速度
    // 注意：这里的系数来源于您之前的代码
    double v_target1 = (-vx_in * 0.667 + vz_final * 0.333) * SPEED_SCALE;
    double v_target2 = ( vx_in * 0.333 + vy_in * 0.577 + vz_final * 0.333) * SPEED_SCALE;
    double v_target3 = ( vx_in * 0.333 - vy_in * 0.577 + vz_final * 0.333) * SPEED_SCALE;
    
    // 全局变量v1,v2,v3用于调试，可以保留
    v1 = v_target1;
    v2 = v_target2;
    v3 = v_target3;

    // 5. 执行各电机速度PID并发送CAN指令
    double motor_out1 = calculate_pid(&PID1, v_target1, motor_chassis[0].speed_rpm, false);
    double motor_out2 = calculate_pid(&PID2, v_target2, motor_chassis[1].speed_rpm, false);
    double motor_out3 = calculate_pid(&PID3, v_target3, motor_chassis[2].speed_rpm, false);

    chassis_can_cmd(motor_out1, motor_out2, motor_out3);
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

/**
 * @brief 底盘停止函数
 * @note  当需要紧急停止或切换到停止模式时调用
 */
void chassis_stop(void)
{
    // 使用通用PID函数进行停止模式控制
    double motor_out1 = calculate_pid(&PID1, 0.0, motor_chassis[0].speed_rpm, true);
    double motor_out2 = calculate_pid(&PID2, 0.0, motor_chassis[1].speed_rpm, true);
    double motor_out3 = calculate_pid(&PID3, 0.0, motor_chassis[2].speed_rpm, true);
    
    // 发送CAN指令
    chassis_can_cmd(motor_out1, motor_out2, motor_out3);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    // 重置航向锁定状态
    heading_lock_first_time = true;
}
