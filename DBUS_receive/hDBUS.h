#ifndef DBUS_H
#define DBUS_H

#include "main.h"

#define BUFF_LEN 18
#define MAX_LEN 36
#define VAL_OFFSET 1024
#define DBUS_UART  huatr3
/*
rocker[0]:ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½×ª ï¿½ï¿½ï¿½ï¿½×ªÎªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
rocker[1]:ï¿½ï¿½Ì¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â½ï¿½ ï¿½ï¿½ï¿½ï¿½Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
rocker[2]:ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
rocker[3]:ï¿½ï¿½ï¿½ï¿½Ç°ï¿½Æºï¿½ï¿½Æ£ï¿½ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½Î§Îª-660 ï¿½ï¿½ +660
*/
typedef struct
{
	int rocker[4];//Ò¡¸Ë
	short sw[2];//sw[0]ÓÒ±ß = 1ÉÏÎ»»ú, 2²»¶¯, 3Ò£¿ØÆ÷£¬ sw[1]×ó±ß = 1, 2·¢Çò, 3µæÇòÄ£Ê½ ¡ª¡ª¡ª¡ª1£¬2£¬3ÉÏÏÂÖÐ
	int roll;
	int key;
	int mod;
	int control_mode;//ÓÒ±ß£º0--É¶¶¼²»¶¯£¬1--ÊÖ±ú²Ù¿Ø, 2ÉÏÎ»»ú²Ù¿Ø
	double pitch;
	int isenable;
} DECODE_VAL;

extern DECODE_VAL DBUS_decode_val;
extern uint8_t DBUS_buff[MAX_LEN];

void decode_buff(uint8_t *DBUS_buff, DECODE_VAL *DBUS_decode_val);//ï¿½ï¿½ï¿½ï¿½
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);//DMA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//IT

#endif // DBUS_H

