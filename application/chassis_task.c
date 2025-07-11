#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "hchassis.h" // Your file with PID, kinematics, etc.
#include "hDBUS.h"
#include "bsp_uart.h"
// 底盘控制任务
void chassis_task(void const *argument)
{
  // 等待其他任务初始化
  osDelay(1000);
  double v2 = 0.14;

    // 初始化PID和CAN
    pid_init();
    can_filter_init();
  // 初始化PID和CAN
  // pid_init();
  // can_filter_init();

  for (;;)
  {
    //				sprintf(uart1_tx_debug_data, "%.2lf\n", v2);
    //				HAL_UART_Transmit_DMA(&huart1, uart1_tx_debug_data, strlen(uart1_tx_debug_data));
    // 根据遥控器的模式切换控制逻辑
    // 模式1: 遥控器控制
    if (DBUS_decode_val.control_mode == 1)
    {
      // 调用新的、集中的底盘控制函数
      chassis_control_task();
    }

    // 模式2: 上位机控制模式
    else if (DBUS_decode_val.control_mode == 2)
    { // 前两个参数为从全场定位读的当前值，后两个参数为上位机发送的目标值
      chassis_control_task2(); //看看能不能跑；
      // chassis_stop();
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
