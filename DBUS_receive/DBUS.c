#include "hDBUS.h"
#include "main.h"
#include "usart.h"

DECODE_VAL DBUS_decode_val = {0};
uint8_t DBUS_buff[MAX_LEN] = {0};


void decode_buff(uint8_t *DBUS_buff, DECODE_VAL *DBUS_decode_val)
{
	if (DBUS_buff == NULL || DBUS_decode_val == NULL) 
    {
        return; 
    } 
		DBUS_decode_val->rocker[0] = (DBUS_buff[0] | (DBUS_buff[1] << 8)) & 0x07ff;                                   //Channel 0 
    DBUS_decode_val->rocker[1] = ((DBUS_buff[1] >> 3) | (DBUS_buff[2] << 5)) & 0x07ff;                        //Channel 1 
    DBUS_decode_val->rocker[2] = ((DBUS_buff[2] >> 6) | (DBUS_buff[3] << 2) | (DBUS_buff[4] << 10)) &0x07ff;  //Channel 2 	
    DBUS_decode_val->rocker[3] = ((DBUS_buff[4] >> 1) | (DBUS_buff[5] << 7)) & 0x07ff;  //!< Channel 3 
    DBUS_decode_val->sw[0] = ((DBUS_buff[5] >> 4) & 0x0003);                                                  //Switch left 
    DBUS_decode_val->sw[1] = ((DBUS_buff[5] >> 4) & 0x000C) >> 2;                                             //Switch right 
    DBUS_decode_val->roll = DBUS_buff[16]|(DBUS_buff[17] << 8);                                                 //roLL
    DBUS_decode_val->rocker[0] -= VAL_OFFSET;
    DBUS_decode_val->rocker[1] -= VAL_OFFSET; 
    DBUS_decode_val->rocker[2] -= VAL_OFFSET; 
    DBUS_decode_val->rocker[3] -= VAL_OFFSET; 
    DBUS_decode_val->roll -= VAL_OFFSET; 
		DBUS_decode_val->key = 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
{
	if(huart == &huart3)
	{
		decode_buff(DBUS_buff,&DBUS_decode_val);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DBUS_buff,Size);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //IT
{

	if(huart == &huart3)
	{
		decode_buff(DBUS_buff,&DBUS_decode_val);
		HAL_UART_Receive_IT(huart,DBUS_buff,BUFF_LEN);
	}
}

