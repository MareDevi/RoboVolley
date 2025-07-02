#ifndef DBUS_H
#define DBUS_H

#include "main.h"

#define BUFF_LEN 18
#define MAX_LEN 36
#define VAL_OFFSET 1024
#define DBUS_UART  huatr3
/*
rocker[0]:底盘左转，右转 向右转为正方向
rocker[1]:云台上升或者下降 向上为正方向
rocker[2]:底盘左移右移，向右是正方向
rocker[3]:底盘前移后移，向前是正方向
范围为-660 到 +660
*/
typedef struct
{
	int rocker[4];//摇杆
	short sw[2];
	int roll;
	int key;
	int mod;
	int over;
	double pitch;
	int delay_tag;
} DECODE_VAL;

extern DECODE_VAL DBUS_decode_val;
extern uint8_t DBUS_buff[MAX_LEN];

void decode_buff(uint8_t *DBUS_buff, DECODE_VAL *DBUS_decode_val);//解码
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);//DMA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//IT

#endif // DBUS_H

