#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "hchassis.h"
#include "hDBUS.h"

// 底盘控制任务
void chassis_task(void const *argument)
{
    osDelay(1000);

    pid_init();
    can_filter_init();

    for (;;)
    {
        if (DBUS_decode_val.control_mode == 1)
        {

            int16_t motor1_out = (int16_t)chassis_motor_1_pid();
            int16_t motor2_out = (int16_t)chassis_motor_2_pid();
            int16_t motor3_out = (int16_t)chassis_motor_3_pid();

            chassis_can_cmd(motor1_out, motor2_out, motor3_out);
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        }
        else if (DBUS_decode_val.control_mode == 2)
        {
            // 通过PID计算但设置target_val=0，保持控制器状态
            int16_t motor1_out = (int16_t)chassis_motor_1_pid_stop();
            int16_t motor2_out = (int16_t)chassis_motor_2_pid_stop();
            int16_t motor3_out = (int16_t)chassis_motor_3_pid_stop();
            
            chassis_can_cmd(motor1_out, motor2_out, motor3_out);
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);

        }
        osDelay(10);
    }
}
