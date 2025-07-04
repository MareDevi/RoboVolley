#ifndef DBUS_H
#define DBUS_H

#include "main.h"

#define BUFF_LEN 18
#define MAX_LEN 36
#define VAL_OFFSET 1024
#define DBUS_UART  huatr3
/*
rocker[0]:������ת����ת ����תΪ������
rocker[1]:��̨���������½� ����Ϊ������
rocker[2]:�����������ƣ�������������
rocker[3]:����ǰ�ƺ��ƣ���ǰ��������
��ΧΪ-660 �� +660
*/
typedef struct
{
	int rocker[4];//ҡ��
	short sw[2];
	int roll;
	int key;
	int mod;
	int control_mode;
	double pitch;
	int delay_tag;
} DECODE_VAL;

extern DECODE_VAL DBUS_decode_val;
extern uint8_t DBUS_buff[MAX_LEN];

void decode_buff(uint8_t *DBUS_buff, DECODE_VAL *DBUS_decode_val);//����
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);//DMA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//IT

#endif // DBUS_H

