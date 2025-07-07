/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hDBUS.h"
#include "usart.h"
#include "vofa.h"
#include "hchassis.h"
#include "stdio.h"
#include "rob2.h"
#include "chassis_task.h"
#include "INS_task.h"
#include "bsp_buzzer.h"
#include "bsp_uart.h"
// #include "hval_out.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId imuTaskHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RcontrolHandle;
osThreadId gimbalTaskHandle;
osThreadId chassisTaskHandle;
osThreadId imuTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Buff_ReCf(void const * argument);
void gimbal(void const * argument);
extern void chassis_task(void const * argument);
extern void INS_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Rcontrol */
  osThreadDef(Rcontrol, Buff_ReCf, osPriorityHigh, 0, 128);
  RcontrolHandle = osThreadCreate(osThread(Rcontrol), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, gimbal, osPriorityIdle, 0, 128);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of chassisTask */
  osThreadDef(chassisTask, chassis_task, osPriorityNormal, 0, 128);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 256);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Buff_ReCf */
/**
 * @brief Function implementing the Rcontrol thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Buff_ReCf */
void Buff_ReCf(void const * argument)
{
  /* USER CODE BEGIN Buff_ReCf */
	/* Infinite loop */
	DBUS_decode_val.isenable = 0;
	static BuzzerState buzzer_state = BUZZER_STATE_IDLE;
	TickType_t xUpTime = xTaskGetTickCount();
	static uint32_t buzzer_start_tick = 0;
	HAL_UART_Receive_IT(&huart3, DBUS_buff, BUFF_LEN);
	// HAL_UARTEx_ReceiveToIdle_IT(&huart3,DBUS_buff,sizeof(DBUS_buff));
	while (1)
	{
		DBUS_decode_val.key = 0;
		osDelay(40);
		if (DBUS_decode_val.key == 0 && DBUS_decode_val.mod == 1)
		{
			DBUS_decode_val.mod = 0;
			// 关电机
			val_clear();
			DBUS_decode_val.control_mode = 0; 
			DBUS_decode_val.pitch = 0;		  
			DBUS_decode_val.isenable = 0;
			Disenable_Motor(&motor5, 0); 
			osDelay(0);
			Disenable_Motor(&motor4, 0); 
			osDelay(0);
			Disenable_Motor(&motor1, 0); 
			osDelay(0);
			Disenable_Motor(&motor2, 0); 
			osDelay(0);
			Disenable_Motor(&motor3, 0); 
			osDelay(0);
			Disenable_Motor(&motor5, 0); 
			osDelay(0);
			// HAL_CAN_Stop(&hcan1);
		}
		else if (DBUS_decode_val.key == 1 && DBUS_decode_val.mod == 0)
		{
			DBUS_decode_val.mod = 1;
			DBUS_decode_val.control_mode = 0;
			// 开电机

			RobStrite_Motor_Init(&motor4, 0x04);
			osDelay(2);
			RobStrite_Motor_Init(&motor1, 0x01);
			RobStrite_Motor_Init(&motor2, 0x02);
			RobStrite_Motor_Init(&motor3, 0x03);
			RobStrite_Motor_Init(&motor5, 0x05);

			Set_RobStrite_Motor_parameter(&motor4, 0x7005, 5, Set_mode);
			osDelay(1);

			Set_RobStrite_Motor_parameter(&motor1, 0x7005, 5, Set_mode);
			Set_RobStrite_Motor_parameter(&motor2, 0x7005, 5, Set_mode);
			osDelay(1);
			Set_RobStrite_Motor_parameter(&motor3, 0x7005, 5, Set_mode);
			Set_RobStrite_Motor_parameter(&motor5, 0x7005, 5, Set_mode);
			osDelay(2);

			Enable_Motor(&motor4);
			osDelay(1);
			Set_ZeroPos(&motor5);
			Set_ZeroPos(&motor1);
			Set_ZeroPos(&motor2);
			Set_ZeroPos(&motor3);

			DBUS_decode_val.pitch = 0;
			pid_init();
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
		}
		if (DBUS_decode_val.control_mode != 0 && DBUS_decode_val.isenable == 0)
		{
			RobStrite_Motor_Pos_control(&motor1, 0.5, 0.02);
			RobStrite_Motor_Pos_control(&motor2, 0.5, 0.02);
			RobStrite_Motor_Pos_control(&motor3, 0.5, 0.02);
			
			RobStrite_Motor_Pos_control(&motor5, 4.0, 0.0); // 要确认下正负
			osDelay(2);
			DBUS_decode_val.isenable = 1;
		}
		osDelay(10);
		if (DBUS_decode_val.key == 1)
		{
			switch (DBUS_decode_val.sw[0])
			{
			case 1:
				DBUS_decode_val.control_mode = 2; // 上位机控制模式
				break;
			case 2:
				DBUS_decode_val.control_mode = 0; // 关机
				break;
			case 3:
				DBUS_decode_val.control_mode = 1; // 遥控器控制模式
				break;
			default:
				break;
			}
			if (DBUS_decode_val.control_mode != 0) 
			{
				switch (buzzer_state) 
				{
						case BUZZER_STATE_IDLE:
								if (DBUS_decode_val.sw[1] == 3) { // 首次检测到 MID
										buzzer_state = BUZZER_STATE_FIRST_UP;
								}
								break;
						
						case BUZZER_STATE_FIRST_UP:
								if (DBUS_decode_val.sw[1] == 1) { // 检测到 UP
										buzzer_state = BUZZER_STATE_MID;
										xUpTime = xTaskGetTickCount();
								} else if (DBUS_decode_val.sw[1] != 3) {
										buzzer_state = BUZZER_STATE_IDLE; // 无效状态重置
								}
								break;
						
						case BUZZER_STATE_MID:
								if (DBUS_decode_val.sw[1] == 3 && xTaskGetTickCount()-xUpTime <= 500) { // 再次检测到 MID
										buzzer_state = BUZZER_STATE_TRIGGERED;
										buzzer_start_tick = xTaskGetTickCount();
										buzzer_on(); // 启动蜂鸣器
										
									//全场定位置零
									
								} else if (DBUS_decode_val.sw[1] != 1 || xTaskGetTickCount()-xUpTime > 500) {
										buzzer_state = BUZZER_STATE_IDLE;
								}
								break;
						
						case BUZZER_STATE_TRIGGERED:
								// 蜂鸣器已触发，由定时器处理关闭
								if (xTaskGetTickCount() - buzzer_start_tick >= 500) 
								{ // 500ms后关闭
										buzzer_off();
										buzzer_state = BUZZER_STATE_IDLE; // 重置状态
								}
								break;
				}
			}
			else
			{
				buzzer_state = BUZZER_STATE_IDLE;
				buzzer_off();
			}
		}
	}

  /* USER CODE END Buff_ReCf */
}

/* USER CODE BEGIN Header_gimbal */
/**
 * @brief Function implementing the gimbalTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gimbal */
void gimbal(void const * argument)
{
  /* USER CODE BEGIN gimbal */
	/* Infinite loop */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xDelay25 = pdMS_TO_TICKS(25); // 25ms 转换为 tick
	const TickType_t xDelay75 = pdMS_TO_TICKS(75); // 75ms 转换为 tick
	const TickType_t xDelay175 = pdMS_TO_TICKS(175); // 175ms 转换为 tick
	const TickType_t xDelay200 = pdMS_TO_TICKS(200); // 200ms 转换为 tick
	int delay_tag = 0;
	int juggle = 0;
	double motor_angle = 0.007;
	double motor_vec = 1.0;
	double final_pitch = 0.0;
	double shot_ball_angle = -0.7; // 要确认正负
	
	while (1)
	{
		if (DBUS_decode_val.control_mode == 1) // 遥控器控制
		{
			DBUS_decode_val.pitch += (-0.000006f * DBUS_decode_val.rocker[1]);
			DBUS_decode_val.pitch = (DBUS_decode_val.pitch > 0.1) ? 0.1 : ((DBUS_decode_val.pitch < -0.8) ? -0.8 : DBUS_decode_val.pitch);
			//将遥控器值映射到pitch轴角度
			
			if(DBUS_decode_val.sw[1] == 3) // 初次进入对颠球模式将云台前倾,后续pitch受遥控器控制，发球板保持竖直不动
			{
				if(juggle == 0)
				{
					juggle = 1;
					DBUS_decode_val.pitch = -0.45;
				}
				shot_ball_angle = 0;
				final_pitch = DBUS_decode_val.pitch;
			}

			RobStrite_Motor_Pos_control(&motor4, 4.0, final_pitch);
			osDelay(1);
			RobStrite_Motor_Pos_control(&motor5, 24.0, shot_ball_angle);
			osDelay(1);
			RobStrite_3Motor_simully_Pos_control(&motor1, &motor2, &motor3, motor_vec, motor_angle);
			osDelay(1);
			switch (delay_tag) 
			{
					case 0: // 默认状态
							if (DBUS_decode_val.sw[1] == 2) // 在发球模式默认状态在限位处
							{
									motor_angle = 0.007;
									motor_vec = 4;
									final_pitch = 0;
									shot_ball_angle = -0.7;
							} 
							else if (DBUS_decode_val.sw[1] == 3) // 在对颠球模式默认状态比限位高一些
							{
									motor_angle = 0.04;
									motor_vec = 16;
									final_pitch = 0;
									shot_ball_angle = 0;
							}
							break;

					case 1: // 低速出限位一点
							motor_angle = 0.04;
							motor_vec = 1;
							final_pitch = 0;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 2;
							break;

					case 2: // 等待25ms电机运行到指定角度
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay25) // 非阻塞式delay
									delay_tag = 3;  // 状态转移 25ms
							break;

					case 3: // 高速击球
							motor_angle = 0.57;
							motor_vec = 24;
							final_pitch = 0;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 4;
							break;

					case 4: // 等待175ms电机运行到指定角度
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay175)
							{
								if(DBUS_decode_val.sw[1] == 3) // 对颠球模式直接回默认状态
									delay_tag = 0;  
								else if(DBUS_decode_val.sw[1] == 2) // 进入发球状态
									delay_tag = 5;  
							}
							break;
							
					case 5: // 回到发球默认状态并将云台向前倾一定角度，防止被发球板打到
							motor_angle = 0.007;
							motor_vec = 4;
							final_pitch = -0.8;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 6;
							break;
					
					case 6: // 等待200ms电机运行到指定角度、排球落到合适的击球点
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay200)
									delay_tag = 7;
							break;
							
					case 7: // 击球
							shot_ball_angle = 0.9; // 要确认正负
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 8;
							break;
					
					case 8: // 等待75ms击球完成
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay75)
									delay_tag = 9;
							break;
							
					case 9: // 击球板回原位
							shot_ball_angle = -0.7;
							final_pitch = 0;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 10;
							break;
					
					case 10: // 等待电机复位
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay200)
									delay_tag = 0;
							break;
					
					default:
							break;
			}
			
			if (DBUS_decode_val.roll >= 600 && delay_tag == 0)
			{
				delay_tag = 1;
			}
			osDelay(3);
		}
		else if (DBUS_decode_val.control_mode == 2) // 上位机控制,发球不用上位机控制
		{
			float final_pitch = (PossiBuffRcf.Pitch > 0.1) ? 0.1 : ((PossiBuffRcf.Pitch < -0.8) ? -0.8 : PossiBuffRcf.Pitch);
			shot_ball_angle = 0;
			RobStrite_Motor_Pos_control(&motor4, 4.0, final_pitch);
			osDelay(1);
			RobStrite_Motor_Pos_control(&motor5, 1.0, shot_ball_angle);
			osDelay(1);
			RobStrite_3Motor_simully_Pos_control(&motor1, &motor2, &motor3, motor_vec, motor_angle);
			osDelay(1);
			switch (delay_tag) 
			{
					case 0: // 默认状态
							motor_angle = 0.04;
							motor_vec = 16;
							final_pitch = 0;
							shot_ball_angle = 0;
							break;

					case 1: // 低速出限位一点
							motor_angle = 0.04;
							motor_vec = 1;
							final_pitch = 0;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 2;
							break;

					case 2: // 等待25ms电机运行到指定角度
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay25) // 非阻塞式delay
									delay_tag = 3;  // 状态转移 25ms
							break;

					case 3: // 高速击球
							motor_angle = 0.57;
							motor_vec = 24;
							final_pitch = 0;
							xLastWakeTime = xTaskGetTickCount();
							delay_tag = 4;
							break;

					case 4: // 等待175ms电机运行到指定角度
							if (xTaskGetTickCount() - xLastWakeTime >= xDelay175)
									delay_tag = 0; // 回默认状态
							break;
					
					default:
							break;
			}
			
			if (PossiBuffRcf.Shot == 1 && delay_tag == 0)
			{
				delay_tag = 1;
			}
			osDelay(3);
		}
		else if (DBUS_decode_val.control_mode == 0)
		{
			if(delay_tag != 0)
			{		
				delay_tag = 0;
				motor_angle = 0.007;
				motor_vec = 1;
				RobStrite_3Motor_simully_Pos_control(&motor1, &motor2, &motor3, motor_vec, motor_angle);
			}
		}
	}
  /* USER CODE END gimbal */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
