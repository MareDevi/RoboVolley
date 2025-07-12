//
#include "INS_task.h"
#include "bsp_uart.h"
#include "can.h"
#include "hDBUS.h"
#include "hchassis.h"
#include "main.h"
#include "math.h"
#include "rob2.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include <math.h>
#include <stdbool.h>

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203

#define SPEED_SCALE 7.8F
#define SPEED_POS 1.0F
#define KR 290.0F

// 声明一个静态变量来保存目标航向角
//static double target_yaw = 0.0;
//static bool heading_lock_first_time = true;
// char uart1_tx_debug_data[300]; // 用于接收串口数据

// PID信息
PID_typedef PID1;
PID_typedef PID2;
PID_typedef PID3;
PID_typedef PID_pos_x;
PID_typedef PID_pos_y;
PID_typedef PID_pos_yaw;

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

  HAL_CAN_AddTxMessage(&hcan1, &chasis_tx_message, chassis_txdata,
                       &send_mail_box);
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
  if (rx_header.IDE == CAN_ID_STD && hcan == &hcan1)
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
  else if (rx_header.IDE == CAN_ID_EXT && hcan == &hcan2)
  {
    // RobStrite_Motor_Analysis(&motor4,rx_data,rx_header.ExtId);
    motor4.Pos_Info.Angle =
        uint16_to_float(rx_data[0] << 8 | rx_data[1], P_MIN, P_MAX, 16);
    PossiBuffSnd.Pitch = motor4.Pos_Info.Angle;
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

  PID_pos_x.kp = KP_P;
  PID_pos_x.ki = KI_P;
  PID_pos_x.kd = KD_P;
  PID_pos_x.out_max = OUT_MAX_p;
  PID_pos_x.iout_max = IOUT_MAX_p;
  PID_pos_x.pout = 0;
  PID_pos_x.iout = 0;
  PID_pos_x.dout = 0;
  PID_pos_x.out = 0;
  PID_pos_x.cur_error = 0;
  PID_pos_x.his_error = 0;

  PID_pos_y.kp = KP_P;
  PID_pos_y.ki = KI_P;
  PID_pos_y.kd = KD_P;
  PID_pos_y.out_max = OUT_MAX_p;
  PID_pos_y.iout_max = IOUT_MAX_p;
  PID_pos_y.pout = 0;
  PID_pos_y.iout = 0;
  PID_pos_y.dout = 0;
  PID_pos_y.out = 0;
  PID_pos_y.cur_error = 0;
  PID_pos_y.his_error = 0;

  PID_pos_yaw.kp = 200.0;
  PID_pos_yaw.ki = 0.0;
  PID_pos_yaw.kd = 0.01;
  PID_pos_yaw.out_max = 1800;
  PID_pos_yaw.iout_max = 5;
  PID_pos_yaw.pout = 0;
  PID_pos_yaw.iout = 0;
  PID_pos_yaw.dout = 0;
  PID_pos_yaw.out = 0;
  PID_pos_yaw.cur_error = 0;
  PID_pos_yaw.his_error = 0;
}

// 简洁通用PID计算函数
static double calculate_pid_simple(PID_typedef *pid, double error,
                                   double deadzone)
{
  // 保存历史误差
  pid->his_error = pid->cur_error;
  pid->cur_error = error;

  // 死区处理
  if (fabs(error) < deadzone)
  {
    pid->cur_error = 0.0;
    pid->iout = 0.0; // 死区内清除积分
    pid->out = 0.0;
    return 0.0;
  }

  // PID计算
  pid->pout = pid->kp * pid->cur_error;
  pid->iout += pid->ki * pid->cur_error;
  pid->dout = pid->kd * (pid->cur_error - pid->his_error);

  // 积分限幅
  if (pid->iout > pid->iout_max)
    pid->iout = pid->iout_max;
  if (pid->iout < -pid->iout_max)
    pid->iout = -pid->iout_max;

  // 计算输出
  pid->out = pid->pout + pid->iout + pid->dout;

  // 输出限幅
  if (pid->out > pid->out_max)
    pid->out = pid->out_max;
  if (pid->out < -pid->out_max)
    pid->out = -pid->out_max;

  return pid->out;
}

// 标准PID：误差 = 目标值 - 当前值
static double calculate_pid_standard(PID_typedef *pid, double target,
                                     double current, double deadzone)
{
  return calculate_pid_simple(pid, target - current, deadzone);
}

// 位置PID：直接使用位置误差
static double calculate_pid_position(PID_typedef *pid, double position_error,
                                     double deadzone)
{
  return calculate_pid_simple(pid, position_error, deadzone);
}

/**
 * @brief 底盘主控制任务，包含航向锁定
 * @note  此函数应在一个循环中被周期性调用
 */
double vx,vy,vz;
void chassis_control_task(void)
{
  // 1. 获取用户输入
  // 遥控器输入，映射到机器人坐标系速度
  double vx_in = DBUS_decode_val.rocker[2]; // 前后速度
  double vy_in = DBUS_decode_val.rocker[3]; // 左右速度
  double vz_in = DBUS_decode_val.rocker[0]; // 旋转速度
	
	vx = vx_in;
	vy = vy_in;
	vz = vz_in;

  // 2. 逆运动学解算
  // 使用 vx_in, vy_in, vz_in 计算三个轮子的目标速度
  double v_target1 = (-vx_in * 0.667 + vz_in * 0.333) * SPEED_SCALE;
  double v_target2 =
      (vx_in * 0.333 + vy_in * 0.577 + vz_in * 0.333) * SPEED_SCALE;
  double v_target3 =
      (vx_in * 0.333 - vy_in * 0.577 + vz_in * 0.333) * SPEED_SCALE;

  // 3. 执行各电机速度PID并发送CAN指令
  double motor_out1 = calculate_pid_standard(&PID1, v_target1,
                                             motor_chassis[0].speed_rpm, false);
  double motor_out2 = calculate_pid_standard(&PID2, v_target2,
                                             motor_chassis[1].speed_rpm, false);
  double motor_out3 = calculate_pid_standard(&PID3, v_target3,
                                             motor_chassis[2].speed_rpm, false);

  //	sprintf((char *)uart1_tx_debug_data, "%d,%d\n", (int)(v_target1 * 100),
  //(int)(motor_chassis[0].speed_rpm * 100)); 	HAL_UART_Transmit_DMA(&huart1,
  //(uint8_t *)uart1_tx_debug_data, strlen(uart1_tx_debug_data));

  chassis_can_cmd(motor_out1, motor_out2, motor_out3);
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

/**
 * @brief 底盘停止函数
 * @note  当需要紧急停止或切换到停止模式时调用
 */
void chassis_stop(void)
{
  // 停止所有电机，使用死区5.0
  double motor_out1 =
      calculate_pid_standard(&PID1, 0.0, motor_chassis[0].speed_rpm, 5.0);
  double motor_out2 =
      calculate_pid_standard(&PID2, 0.0, motor_chassis[1].speed_rpm, 5.0);
  double motor_out3 =
      calculate_pid_standard(&PID3, 0.0, motor_chassis[2].speed_rpm, 5.0);

  chassis_can_cmd(motor_out1, motor_out2, motor_out3);
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

/**
 * @brief 全场定位
 * @note 控制电机到达目标位置
 */

// 定义调试用全局变量
#define X_TARGET 0.0F
#define Y_TARGET 0.0F
#define YAW_TARGET 0.0F
float position_x_target;
float position_x_current;
float position_y_target;
float position_y_current;
float position_yaw_target;
float position_yaw_current;

float position_x_Etarget;
float position_x_Ecurrent;
float position_y_Etarget;
float position_y_Ecurrent;

float delta_x_absolute;
float delta_y_absolute;
float delta_yaw;
float position_yaw;
float alpha;
float beta;
float len;
float delta_x_equal;
float delta_y_equal;
double vx_in_posi;
double vy_in_posi;
double vz_in_posi;

//new
int state = 0;

void chassis_auto_task(void) // 视觉定位任务
{
  // 1. 获取位置数据
  position_x_current = -position.world_y;   // 全场定位X坐标
  position_y_current = -position.world_x;   // 全场定位Y坐标
	position_yaw = -position.world_yaw / 180  *3.1415; // 全场定位航向角(spi读的航向角是角度制)

  // 2. 设置目标位置（从上位机获取 PossiBuffRcf，调试时使用固定值）
  position_x_target = 200.0;//PossiBuffRcf.X;     // 目标X坐标，调试时可用 X_TARGET
  position_y_target = 600.0;//PossiBuffRcf.Y;     // 目标Y坐标，调试时可用 Y_TARGET  
  position_yaw_target = 1.57; //PossiBuffRcf.Yaw; // 目标航向角，调试时可用 YAW_TARGET
  
  // 3. 计算全局位置误差
  delta_yaw = position_yaw - position_yaw_target;

  // 镝神的坐标变换
  position_x_Etarget = position_x_target*cos(position_yaw) - position_y_target*sin(position_yaw);
  position_y_Etarget = position_x_target*sin(position_yaw) + position_y_target*cos(position_yaw);
  position_x_Ecurrent = position_x_current*cos(position_yaw) - position_y_current*sin(position_yaw);
  position_y_Ecurrent = position_x_current*sin(position_yaw) + position_y_current*cos(position_yaw);

  delta_x_equal = position_x_Ecurrent - position_x_Etarget;
  delta_y_equal = position_y_Ecurrent - position_y_Etarget;
	// 4. 状态机控制：先平移到位，再旋转
	switch(state)
	{
		case 0: // 平移阶段 - 只控制X和Y轴
			// 检查是否到达位置（20mm误差范围内）
			if(fabs(delta_x_equal) < 50 && fabs(delta_y_equal) < 50)
			{
				state = 1; // 切换到旋转阶段
			}
			// 只控制平移，不控制旋转
			vx_in_posi = -calculate_pid_position(&PID_pos_x, delta_x_equal, 10.0) * SPEED_POS;
			vy_in_posi = -calculate_pid_position(&PID_pos_y, delta_y_equal, 10.0) * SPEED_POS;
			vz_in_posi = 0.0; // 旋转速度为0
			break;
			
		case 1: // 旋转阶段 - 只控制Yaw轴
			// 检查是否旋转到位
			if(fabs(delta_yaw) < 0.05)
			{
				state = 2; // 切换到完成状态
			}
			// 保持位置，只控制旋转
			vx_in_posi = 0; //-calculate_pid_position(&PID_pos_x, delta_x_equal, 10.0) * SPEED_POS * 0.3; // 位置保持（降低增益）
			vy_in_posi = 0; //-calculate_pid_position(&PID_pos_y, delta_y_equal, 10.0) * SPEED_POS * 0.3; // 位置保持（降低增益）
			vz_in_posi = -calculate_pid_position(&PID_pos_yaw, delta_yaw, 0.05) * SPEED_POS; // 旋转控制
			break;
			
		case 2: // 完成状态 - 全部控制以保持位置
			// 所有轴都进行精确控制
			state = 0;
			break;
			
		default:
			// 异常状态，停止所有运动
			vx_in_posi = 0.0;
			vy_in_posi = 0.0;
			vz_in_posi = 0.0;
			state = 0; // 重置状态
			break;
	}
  
  // 6. 逆运动学解算（包含旋转分量）
  double v_target1 = (-vx_in_posi * 0.667 + vz_in_posi * 0.333) * SPEED_SCALE;
  double v_target2 = (vx_in_posi * 0.333 + vy_in_posi * 0.577 + vz_in_posi * 0.333) * SPEED_SCALE;
  double v_target3 = (vx_in_posi * 0.333 - vy_in_posi * 0.577 + vz_in_posi * 0.333) * SPEED_SCALE;

  // 7. 电机速度PID控制
  double motor_out1 = calculate_pid_standard(&PID1, v_target1, motor_chassis[0].speed_rpm, 10.0);
  double motor_out2 = calculate_pid_standard(&PID2, v_target2, motor_chassis[1].speed_rpm, 10.0);
  double motor_out3 = calculate_pid_standard(&PID3, v_target3, motor_chassis[2].speed_rpm, 10.0);

  // 8. 发送控制指令
  chassis_can_cmd(motor_out1, motor_out2, motor_out3);
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
