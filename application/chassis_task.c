#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "hchassis.h"

// 底盘控制任务
void chassis_task(void const *argument)
{
    osDelay(1000);

    pid_init();
    can_filter_init();

    for(;;)
    {
        int16_t motor1_out = (int16_t)chassis_motor_1_pid_v2();
        int16_t motor2_out = (int16_t)chassis_motor_2_pid_v2();
        int16_t motor3_out = (int16_t)chassis_motor_3_pid_v2();

        chassis_can_cmd(motor1_out, motor2_out, motor3_out);

        osDelay(10);
    }
}
