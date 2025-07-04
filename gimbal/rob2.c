#include "rob2.h"
#include <string.h>
#include "hDBUS.h"

// 定义
extern CAN_HandleTypeDef hcan2;
uint32_t Mailbox;
RobStrite_Motor motor1;
RobStrite_Motor motor2;
RobStrite_Motor motor3;
RobStrite_Motor motor4;

// 类型变换
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Byte_to_float(uint8_t* bytedata) {
    uint32_t data = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
    return *(float*)&data;
}

// 初始化
void RobStrite_Motor_Init(RobStrite_Motor* motor, uint8_t CAN_Id) {
    motor->CAN_ID = CAN_Id;
    motor->Master_CAN_ID = 0xFD;
    motor->Motor_Set_All.set_motor_mode = move_control_mode;
    motor->Motor_Set_All.output = 0.0f;  
    motor->Motor_Offset_MotoFunc = NULL;
    motor->error_code = 0;
    
    static const uint16_t default_index_list[15] = {
        0x2001, 0x2002, 0x2003, 0x2004, 0x2005,
        0x2006, 0x2007, 0x2008, 0x2009, 0x200A,
        0x200B, 0x200C, 0x200D, 0x200E, 0x200F
    };
    
    memcpy(motor->Index_List, default_index_list, sizeof(default_index_list));
    DataReadWrite_Init(&motor->drw, motor->Index_List);
}

void RobStrite_Motor_Init_With_Offset(RobStrite_Motor* motor, float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id) {
    RobStrite_Motor_Init(motor, CAN_Id);
    motor->Motor_Offset_MotoFunc = Offset_MotoFunc;
}

void DataReadWrite_Init(DataReadWrite* drw, const uint16_t* index_list) {
    drw->run_mode.index = index_list[0];
    drw->iq_ref.index = index_list[1];
    drw->spd_ref.index = index_list[2];
    drw->imit_torque.index = index_list[3];
    drw->cur_kp.index = index_list[4];
    drw->cur_ki.index = index_list[5];
    drw->cur_filt_gain.index = index_list[6];
    drw->loc_ref.index = index_list[7];
    drw->limit_spd.index = index_list[8];
    drw->limit_cur.index = index_list[9];
    drw->mechPos.index = index_list[10];
    drw->iqf.index = index_list[11];
    drw->mechVel.index = index_list[12];
    drw->VBUS.index = index_list[13];
    drw->rotation.index = index_list[14];
}

// 数据分析
void RobStrite_Motor_Analysis(RobStrite_Motor* motor, uint8_t* DataFrame, uint32_t ID_ExtId) {
    if ((uint8_t)((ID_ExtId & 0xFF00) >> 8) == motor->CAN_ID) {
        if ((int)((ID_ExtId & 0x3F000000) >> 24) == 2) {
            motor->Pos_Info.Angle = uint16_to_float(DataFrame[0] << 8 | DataFrame[1], P_MIN, P_MAX, 16);
            motor->Pos_Info.Speed = uint16_to_float(DataFrame[2] << 8 | DataFrame[3], V_MIN, V_MAX, 16);
            motor->Pos_Info.Torque = uint16_to_float(DataFrame[4] << 8 | DataFrame[5], T_MIN, T_MAX, 16);
            motor->Pos_Info.Temp = (DataFrame[6] << 8 | DataFrame[7]) * 0.1;
            motor->error_code = (uint8_t)((ID_ExtId & 0x3F0000) >> 16);
            motor->Pos_Info.pattern = (uint8_t)((ID_ExtId & 0xC00000) >> 22);
        }
        else if ((int)((ID_ExtId & 0x3F000000) >> 24) == 17) {
            for (int index_num = 0; index_num <= 13; index_num++) {
                if ((DataFrame[1] << 8 | DataFrame[0]) == motor->Index_List[index_num]) {
                    switch (index_num) {
                        case 0:
                            motor->drw.run_mode.data = (uint8_t)DataFrame[4];
                            break;
                        case 1:
                            motor->drw.iq_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 2:
                            motor->drw.spd_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 3:
                            motor->drw.imit_torque.data = Byte_to_float(DataFrame);
                            break;
                        case 4:
                            motor->drw.cur_kp.data = Byte_to_float(DataFrame);
                            break;
                        case 5:
                            motor->drw.cur_ki.data = Byte_to_float(DataFrame);
                            break;
                        case 6:
                            motor->drw.cur_filt_gain.data = Byte_to_float(DataFrame);
                            break;
                        case 7:
                            motor->drw.loc_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 8:
                            motor->drw.limit_spd.data = Byte_to_float(DataFrame);
                            break;
                        case 9:
                            motor->drw.limit_cur.data = Byte_to_float(DataFrame);
                            break;
                        case 10:
                            motor->drw.mechPos.data = Byte_to_float(DataFrame);
                            break;
                        case 11:
                            motor->drw.iqf.data = Byte_to_float(DataFrame);
                            break;
                        case 12:
                            motor->drw.mechVel.data = Byte_to_float(DataFrame);
                            break;
                        case 13:
                            motor->drw.VBUS.data = Byte_to_float(DataFrame);
                            break;
                    }
                }
            }
        }
        else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE) {
            motor->CAN_ID = (uint8_t)((ID_ExtId & 0xFF00) >> 8);
        }
    }
}

// 获得CAN ID
void RobStrite_Get_CAN_ID(RobStrite_Motor* motor) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_Get_ID << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 运控模式
void RobStrite_Motor_move_control(RobStrite_Motor* motor, float Torque, float Angle, float Speed, float Kp, float Kd) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    motor->Motor_Set_All.set_Torque = Torque;
    motor->Motor_Set_All.set_angle = Angle;
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_Kp = Kp;
    motor->Motor_Set_All.set_Kd = Kd;
    
    if (motor->drw.run_mode.data != 0 && motor->Pos_Info.pattern == 2) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, move_control_mode, Set_mode);
        Get_RobStrite_Motor_parameter(motor, 0x7005);
        motor->Motor_Set_All.set_motor_mode = move_control_mode;
    }
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_MotionControl << 24 | 
                      float_to_uint(motor->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16) >> 8;
    txdata[1] = float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
    txdata[2] = float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16) >> 8;
    txdata[3] = float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
    txdata[4] = float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16) >> 8;
    txdata[5] = float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16);
    txdata[6] = float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16) >> 8;
    txdata[7] = float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16);
    
    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 位置模式
void RobStrite_Motor_Pos_control(RobStrite_Motor* motor, float Speed, float Angle) {
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_angle = Angle;
    
    if (motor->drw.run_mode.data != 1 && motor->Pos_Info.pattern == 2) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, Pos_control_mode, Set_mode);
        Get_RobStrite_Motor_parameter(motor, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Pos_control_mode;
    }
    
    Set_RobStrite_Motor_parameter(motor, 0x7017, motor->Motor_Set_All.set_speed, Set_parameter);
    //Set_RobStrite_Motor_parameter(motor, 0x7025, motor->Motor_Set_All.set_acceleration, Set_parameter);
    Set_RobStrite_Motor_parameter(motor, 0x7016, motor->Motor_Set_All.set_angle, Set_parameter);
}

// 速度模式
uint8_t count_set_motor_mode_Speed = 0;
void RobStrite_Motor_Speed_control(RobStrite_Motor* motor, float Speed, float acceleration, float limit_cur) {
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_limit_cur = limit_cur;
    motor->Motor_Set_All.set_acceleration = acceleration;
    
    if (motor->Pos_Info.pattern == 2) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, Speed_control_mode, Set_mode);
        motor->Motor_Set_All.set_motor_mode = Speed_control_mode;
    }
    
    count_set_motor_mode_Speed++;
    if (count_set_motor_mode_Speed % 2 == 0) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, Speed_control_mode, Set_mode);
    }
    
    Set_RobStrite_Motor_parameter(motor, 0x7018, motor->Motor_Set_All.set_limit_cur, Set_parameter);
    Set_RobStrite_Motor_parameter(motor, 0x7022, motor->Motor_Set_All.set_acceleration, Set_parameter);
    Set_RobStrite_Motor_parameter(motor, 0x700A, motor->Motor_Set_All.set_speed, Set_parameter);
}

// 电流模式
uint8_t count_set_motor_mode = 0;
void RobStrite_Motor_current_control(RobStrite_Motor* motor, float current) {
    motor->Motor_Set_All.set_current = current;
    motor->Motor_Set_All.output = current;  
    
    if (motor->Pos_Info.pattern == 2 && motor->Motor_Set_All.set_motor_mode != 3) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, Elect_control_mode, Set_mode);
        motor->Motor_Set_All.set_motor_mode = Elect_control_mode;
    }
    
    count_set_motor_mode++;
    if (count_set_motor_mode % 50 == 0) {
        Set_RobStrite_Motor_parameter(motor, 0x7005, Elect_control_mode, Set_mode);
    }
    
    Set_RobStrite_Motor_parameter(motor, 0x7006, motor->Motor_Set_All.set_current, Set_parameter);
}

// 零位模式
void RobStrite_Motor_Set_Zero_control(RobStrite_Motor* motor) {
    Set_RobStrite_Motor_parameter(motor, 0x7005, Set_Zero_mode, Set_mode);
}

// 使能
void Enable_Motor(RobStrite_Motor* motor) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_MotorEnable << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 失能
void Disenable_Motor(RobStrite_Motor* motor, uint8_t clear_error) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    txdata[0] = clear_error;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_MotorStop << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 单个参数发送
void Set_RobStrite_Motor_parameter(RobStrite_Motor* motor, uint16_t Index, float Value, char Value_mode) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_SetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    
    txdata[0] = Index;
    txdata[1] = Index >> 8;
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    
    if (Value_mode == Set_parameter) {
        memcpy(&txdata[4], &Value, 4);
    } else if (Value_mode == Set_mode) {
        motor->Motor_Set_All.set_motor_mode = (int)Value;
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }
		
    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
		//HAL_Delay(1);
}

//单个参数接收
void Get_RobStrite_Motor_parameter(RobStrite_Motor* motor, uint16_t Index) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    txdata[0] = Index;
    txdata[1] = Index >> 8;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_GetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 设置CAN ID
void Set_CAN_ID(RobStrite_Motor* motor, uint8_t Set_CAN_ID) {
    Disenable_Motor(motor, 0);
    
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_Can_ID << 24 | Set_CAN_ID << 16 | motor->Master_CAN_ID << 8 | motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
}

// 设置零位
void Set_ZeroPos(RobStrite_Motor* motor) {
    Disenable_Motor(motor, 0);
    
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_SetPosZero << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
    
    txdata[0] = 1;
    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
    
    Enable_Motor(motor);
}

void Set_RobStrite_3Motor_simully_parameter(RobStrite_Motor* motor1, RobStrite_Motor* motor2, RobStrite_Motor* motor3, uint16_t Index, float Value, char Value_mode) {
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
		
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    
    txdata[0] = Index;
    txdata[1] = Index >> 8;
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    
    if (Value_mode == Set_parameter) {
        memcpy(&txdata[4], &Value, 4);
    } else if (Value_mode == Set_mode) {
        motor1->Motor_Set_All.set_motor_mode = (int)Value;
				motor2->Motor_Set_All.set_motor_mode = (int)Value;
				motor3->Motor_Set_All.set_motor_mode = (int)Value;	
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }
    
    TxMessage.ExtId = Communication_Type_SetSingleParameter << 24 | motor1->Master_CAN_ID << 8 | motor1->CAN_ID;
		HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
		HAL_Delay(1);
		
		TxMessage.ExtId = Communication_Type_SetSingleParameter << 24 | motor2->Master_CAN_ID << 8 | motor2->CAN_ID;
		HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
		HAL_Delay(1);

		TxMessage.ExtId = Communication_Type_SetSingleParameter << 24 | motor3->Master_CAN_ID << 8 | motor3->CAN_ID;
    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, txdata, &Mailbox);
		HAL_Delay(1);
		
}

//三个电机控制位置函数
void RobStrite_3Motor_simully_Pos_control(RobStrite_Motor* motor1,RobStrite_Motor* motor2,RobStrite_Motor* motor3, float Speed, float Angle) {
    motor1->Motor_Set_All.set_speed = Speed;
    motor1->Motor_Set_All.set_angle = Angle;
    
    Set_RobStrite_3Motor_simully_parameter(motor1,motor2,motor3,0x7017,motor1->Motor_Set_All.set_speed,Set_parameter);
	
	  Set_RobStrite_3Motor_simully_parameter(motor1,motor2,motor3,0x7016,motor1->Motor_Set_All.set_angle,Set_parameter);
}