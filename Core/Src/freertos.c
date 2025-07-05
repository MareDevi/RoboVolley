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
//#include "hval_out.h"
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
osThreadId delayTaskHandle;
osThreadId chassisTaskHandle;
osThreadId imuTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Buff_ReCf(void const * argument);
void gimbal(void const * argument);
void delay_for_platform(void const * argument);
extern void chassis_task(void const * argument);
extern void INS_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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

  /* definition and creation of delayTask */
  osThreadDef(delayTask, delay_for_platform, osPriorityIdle, 0, 128);
  delayTaskHandle = osThreadCreate(osThread(delayTask), NULL);

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
  for(;;)
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
	HAL_UART_Receive_IT(&huart3,DBUS_buff,BUFF_LEN);
	//HAL_UARTEx_ReceiveToIdle_IT(&huart3,DBUS_buff,sizeof(DBUS_buff));
  while(1)
  {
		DBUS_decode_val.key = 0;
    osDelay(40);
		if(DBUS_decode_val.key == 0 && DBUS_decode_val.mod == 1)
		{
			DBUS_decode_val.mod = 0;
			//关电机
			val_clear();
			DBUS_decode_val.control_mode = 0;//
			DBUS_decode_val.pitch = 0;//
			DBUS_decode_val.delay_tag = 0;//
			DBUS_decode_val.isenable = 0;
			Disenable_Motor(&motor4,0);//
			osDelay(0);
			Disenable_Motor(&motor1,0);//
			osDelay(0);
			Disenable_Motor(&motor2,0);//
			osDelay(0);
		  Disenable_Motor(&motor3,0);//
			osDelay(0);
			//HAL_CAN_Stop(&hcan1);		
		}
		else if(DBUS_decode_val.key == 1 && DBUS_decode_val.mod == 0)
		{
			DBUS_decode_val.mod = 1;
			DBUS_decode_val.control_mode = 0;
			//开电机
			can_filter_init();
			
			RobStrite_Motor_Init(&motor4, 0x04);
			osDelay(2);
			RobStrite_Motor_Init(&motor1, 0x01);
			RobStrite_Motor_Init(&motor2, 0x02);
			RobStrite_Motor_Init(&motor3, 0x03);
			
			Set_RobStrite_Motor_parameter(&motor4, 0x7005, 5, Set_mode);
			osDelay(2);
			//Set_RobStrite_3Motor_simully_parameter(&motor1,&motor2,&motor3,0x7005,5,Set_mode);
			Set_RobStrite_Motor_parameter(&motor1, 0x7005, 5, Set_mode);
			Set_RobStrite_Motor_parameter(&motor2, 0x7005, 5, Set_mode);
			Set_RobStrite_Motor_parameter(&motor3, 0x7005, 5, Set_mode);
			osDelay(2);
			
			Enable_Motor(&motor4);//
			osDelay(1);
			//Enable_Motor(&motor1);
			Set_ZeroPos(&motor1);
			//osDelay(1);
			//Enable_Motor(&motor2);
			Set_ZeroPos(&motor2);
			//osDelay(1);
			//Enable_Motor(&motor3);
			Set_ZeroPos(&motor3);
			//osDelay(2);
			
			DBUS_decode_val.pitch = 0;
			pid_init();
		}
		if(DBUS_decode_val.control_mode != 0 && DBUS_decode_val.isenable == 0)
		{
			RobStrite_Motor_Pos_control(&motor1,0.5,0.02);
			RobStrite_Motor_Pos_control(&motor2,0.5,0.02);
			RobStrite_Motor_Pos_control(&motor3,0.5,0.02);
			osDelay(2);
			
			
			// Set_RobStrite_Motor_parameter(&motor1, 0x7005, 3, Set_mode);
			// Set_RobStrite_Motor_parameter(&motor2, 0x7005, 3, Set_mode);
			// Set_RobStrite_Motor_parameter(&motor3, 0x7005, 3, Set_mode);
			// osDelay(2);
			
			DBUS_decode_val.isenable = 1;
		}
		osDelay(10);
		if(DBUS_decode_val.key == 1)
		{
			switch (DBUS_decode_val.sw[0])
			{
				case 1:
					DBUS_decode_val.control_mode = 2;//上位机控制模式
					break;
				case 2:
					DBUS_decode_val.control_mode = 0;//关机
					break;
				case 3:
					DBUS_decode_val.control_mode = 1;//遥控器控制模式
					break;
				default:
					break;
			}
			//chassis_can_cmd(DBUS_decode_val.sw[0],DBUS_decode_val.sw[1],DBUS_decode_val.control_mode);
			if(DBUS_decode_val.control_mode != 0 && DBUS_decode_val.sw[1] == 1)
			{
				osDelay(100);
				if(DBUS_decode_val.sw[1] == 3)
				{
					HAL_TIM_Base_Start(&htim4);
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
					buzzer_on(0, 10000);
					osDelay(500);
					HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
				}
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
	DBUS_decode_val.delay_tag = 0;
	DBUS_decode_val.bounce_mode = 0;
	DBUS_decode_val.bounce_time = 0;
	double motor1_angle = 0.02;
	double motor1_vec = 1.0;
  while(1)
	{
		if(DBUS_decode_val.control_mode == 1)
		{
			DBUS_decode_val.pitch += (-0.000008f * DBUS_decode_val.rocker[1]);
			if(DBUS_decode_val.pitch >= 0.1)
					DBUS_decode_val.pitch = 0.1;
			if(DBUS_decode_val.pitch <= -0.8)
					DBUS_decode_val.pitch = -0.8;	
			RobStrite_Motor_Pos_control(&motor4, 1.2, DBUS_decode_val.pitch);
			osDelay(1);
			
			if(DBUS_decode_val.delay_tag == 1)
			{  
				motor1_angle = 0.25;
				motor1_vec = /*1*/0.5;
				RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,motor1_vec,motor1_angle);
				osDelay(25);
				DBUS_decode_val.delay_tag =2;
			}
			else if(DBUS_decode_val.delay_tag == 0 && DBUS_decode_val.bounce_mode == 0/*&& DBUS_decode_val.roll < 600*/)
			{
				motor1_angle = 0.005;
				motor1_vec = /*6*/3;
				RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,motor1_vec,motor1_angle);
				osDelay(0);	
			}
			else if(DBUS_decode_val.delay_tag == 0 && DBUS_decode_val.bounce_mode == 1)
			{
				motor1_angle = 0.03875;
				motor1_vec = /*8*/4;
				RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,motor1_vec,motor1_angle);
				osDelay(0);	
				if(DBUS_decode_val.bounce_time++ - 100 > 0)
				{
					DBUS_decode_val.bounce_mode = 0;
					DBUS_decode_val.bounce_time = 0;
				}
			}
			if(DBUS_decode_val.roll >= 600 && DBUS_decode_val.delay_tag == 0)
			{
				DBUS_decode_val.delay_tag = 1;
				DBUS_decode_val.bounce_mode = 1;
			}
			osDelay(3);
		}
		else if(DBUS_decode_val.control_mode == 2)//上位机控制
		{
			;
		}
		
	}
  /* USER CODE END gimbal */
}

/* USER CODE BEGIN Header_delay_for_platform */
/**
* @brief Function implementing the delayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_delay_for_platform */
void delay_for_platform(void const * argument)
{
  /* USER CODE BEGIN delay_for_platform */
  /* Infinite loop */
  while(1)
	{
		if(DBUS_decode_val.delay_tag == 2)//ڽ׈̬
		{
			RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,/*6*/1.5,0.35);
			osDelay(25);
			DBUS_decode_val.delay_tag = 3;
		}
		else if (DBUS_decode_val.delay_tag == 3)
		{
			RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,/*16*/4,0.45);
			osDelay(25);
			DBUS_decode_val.delay_tag = 4;
		}
		else if(DBUS_decode_val.delay_tag == 4)//ݓ̙ʏʽ
		{
			RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,/*24*/6,0.57);
			osDelay(25);
			DBUS_decode_val.delay_tag = 5;
		}
		else if(DBUS_decode_val.delay_tag == 5)//՚خ֥ʏݵ̙ìѣԖψ֨є
		{
			RobStrite_3Motor_simully_Pos_control(&motor1,&motor2,&motor3,/*3*/1.5,0.59);
			osDelay(100);
			DBUS_decode_val.delay_tag = 0;
		}
		osDelay(3);
	}
  /* USER CODE END delay_for_platform */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
