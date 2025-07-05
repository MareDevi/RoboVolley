#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "hchassis.h"
#include "hDBUS.h"

// 底盘控制任务
// void chassis_task(void const *argument)
// {
//     osDelay(1000);

//     pid_init();
//     can_filter_init();

//     for (;;)
//     {
//         if (DBUS_decode_val.control_mode == 1)
//         {

//             int16_t motor1_out = (int16_t)chassis_motor_1_pid();
//             int16_t motor2_out = (int16_t)chassis_motor_2_pid();
//             int16_t motor3_out = (int16_t)chassis_motor_3_pid();

//             chassis_can_cmd(motor1_out, motor2_out, motor3_out);
//             HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
//         }
//         else if (DBUS_decode_val.control_mode == 2)
//         {
//             // 通过PID计算但设置target_val=0，保持控制器状态
//             int16_t motor1_out = (int16_t)chassis_motor_1_pid_stop();
//             int16_t motor2_out = (int16_t)chassis_motor_2_pid_stop();
//             int16_t motor3_out = (int16_t)chassis_motor_3_pid_stop();
            
//             chassis_can_cmd(motor1_out, motor2_out, motor3_out);
//             HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);

//         }
//         osDelay(10);
//     }
// }

// #include "chassis_task.h"
// #include "main.h"
// #include "cmsis_os.h"
// #include "hchassis.h" // Your file with PID, kinematics, etc.
// #include "hDBUS.h"

// 底盘控制任务
void chassis_task(void const *argument)
{
    // 等待其他任务初始化
    osDelay(1000);

    // 初始化PID和CAN
    pid_init();
    can_filter_init();

    for (;;)
    {
        // 根据遥控器的模式切换控制逻辑
        // 模式1: 遥控器控制
        if (DBUS_decode_val.control_mode == 1)
        {
            // 调用新的、集中的底盘控制函数
            chassis_control_task(); 
        }
        // 模式2: 停止/刹车模式
        else if (DBUS_decode_val.control_mode == 2)
        {
            // 调用新的刹车函数
            chassis_stop();
        }
        // 模式3 (或其他): 完全断电/失能模式
        else
        {
            // 直接发送0指令，确保电机停止
            chassis_can_cmd(0, 0, 0);
        }

        // 任务延时，决定控制频率 (10ms -> 100Hz)
        osDelay(10);
    }
}