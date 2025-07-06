#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
void buzzer_on()
{
		HAL_TIM_Base_Start(&htim4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_PRESCALER(&htim4, 0);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);

}

void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
