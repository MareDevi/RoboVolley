#ifndef chassis_H
#define chassis_H

#include "main.h"
#include "can.h"
#include "pid.h"

#define KP 10.5F
#define KI 0.6F
#define KD 0.3F
#define IOUT_MAX 3000.0F
#define OUT_MAX 18000.0F

#define KP_P 0.5F
#define KI_P 0.0F
#define KD_P 0.0F
#define IOUT_MAX_p 100.0F
#define OUT_MAX_p 500.0F

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;

extern motor_measure_t motor_chassis[3];

typedef struct
{
	double kp;
	double ki;
	double kd;
	double pout;
	double iout;
	double dout;
	double out;
	double iout_max;
	double out_max;
	double cur_error;
	double his_error;
} PID_typedef;

extern PID_typedef PID1, PID2, PID3;

// 初始化CAN滤波器
void can_filter_init(void);
// 初始化PID控制器
void pid_init(void);
// 发送底盘电机控制指令
void chassis_can_cmd(int16_t motor1, int16_t motor2, int16_t motor3);

// 底盘控制任务（包含航向锁定功能）
void chassis_control_task(void);
// 底盘停止控制
void chassis_stop(void);
// 关闭电机断电
void shut_up(void);
// 清除数值
void val_clear(void);
// 位置环控制
void chassis_navi(float x_now, float y_now, float x_tar, float y_tar);

void chassis_auto_task(void); // 定位跑位
// 已弃用的单独电机PID函数声明（已在chassis.c中注释掉）
// double chassis_motor_1_pid(void);
// double chassis_motor_2_pid(void);
// double chassis_motor_3_pid(void);
// double chassis_motor_1_pid_stop(void);
// double chassis_motor_2_pid_stop(void);
// double chassis_motor_3_pid_stop(void);

#endif //
