#ifndef __ROB2_H__
#define __ROB2_H__

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>

// 常量定义
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

// 通信类型定义
#define Communication_Type_Get_ID 0
#define Communication_Type_MotionControl 1
#define Communication_Type_MotorEnable 3
#define Communication_Type_MotorStop 4
#define Communication_Type_SetSingleParameter 18
#define Communication_Type_GetSingleParameter 17
#define Communication_Type_Can_ID 7
#define Communication_Type_SetPosZero 6

// 控制模式定义
#define move_control_mode 0
#define Pos_control_mode 5
#define Speed_control_mode 2
#define Elect_control_mode 3
#define Set_Zero_mode 4

// 数据读写模式定义
#define Set_mode 'j'
#define Set_parameter 'p'

// 数据结构定义
typedef struct {
    uint16_t index;
    float data;
} DataEntry;

typedef struct {
    DataEntry run_mode;
    DataEntry iq_ref;
    DataEntry spd_ref;
    DataEntry imit_torque;
    DataEntry cur_kp;
    DataEntry cur_ki;
    DataEntry cur_filt_gain;
    DataEntry loc_ref;
    DataEntry limit_spd;
    DataEntry limit_cur;
    DataEntry mechPos;
    DataEntry iqf;
    DataEntry mechVel;
    DataEntry VBUS;
    DataEntry rotation;
} DataReadWrite;

typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    uint8_t pattern;
} MotorPosInfo;

typedef struct {
    float set_motor_mode;
    float set_Torque;
    float set_angle;
    float set_speed;
    float set_Kp;
    float set_Kd;
    float set_current;
    float set_limit_cur;
    float set_acceleration;
    float output;  // 添加output成员
} MotorSetAll;

// 移除重复定义，使用STM32 HAL库中的定义
// typedef struct {
//     uint8_t IDE;
//     uint8_t RTR;
//     uint8_t DLC;
//     uint32_t ExtId;
// } CAN_TxHeaderTypeDef;

// 电机控制结构体
typedef struct {
    uint8_t CAN_ID;
    uint8_t Master_CAN_ID;
    MotorPosInfo Pos_Info;
    MotorSetAll Motor_Set_All;
    DataReadWrite drw;
    uint8_t error_code;
    float (*Motor_Offset_MotoFunc)(float Motor_Tar);
    uint16_t Index_List[15];
} RobStrite_Motor;

// 函数声明
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float Byte_to_float(uint8_t* bytedata);

void RobStrite_Motor_Init(RobStrite_Motor* motor, uint8_t CAN_Id);
void RobStrite_Motor_Init_With_Offset(RobStrite_Motor* motor, float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id);
void DataReadWrite_Init(DataReadWrite* drw, const uint16_t* index_list);

void RobStrite_Motor_Analysis(RobStrite_Motor* motor, uint8_t* DataFrame, uint32_t ID_ExtId);
void RobStrite_Get_CAN_ID(RobStrite_Motor* motor);
void RobStrite_Motor_move_control(RobStrite_Motor* motor, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStrite_Motor_Pos_control(RobStrite_Motor* motor, float Speed, float Angle);
void RobStrite_Motor_Speed_control(RobStrite_Motor* motor, float Speed, float acceleration, float limit_cur);
void RobStrite_Motor_current_control(RobStrite_Motor* motor, float current);
void RobStrite_Motor_Set_Zero_control(RobStrite_Motor* motor);
void Enable_Motor(RobStrite_Motor* motor);
void Disenable_Motor(RobStrite_Motor* motor, uint8_t clear_error);
void Set_RobStrite_Motor_parameter(RobStrite_Motor* motor, uint16_t Index, float Value, char Value_mode);
void Get_RobStrite_Motor_parameter(RobStrite_Motor* motor, uint16_t Index);
void Set_CAN_ID(RobStrite_Motor* motor, uint8_t Set_CAN_ID);
void Set_ZeroPos(RobStrite_Motor* motor);
#endif  //__ROBSTRITE_H__

#ifndef SHARED_H
#define SHARED_H
extern RobStrite_Motor motor1;
extern RobStrite_Motor motor2;
extern RobStrite_Motor motor3;
extern RobStrite_Motor motor4;
#endif