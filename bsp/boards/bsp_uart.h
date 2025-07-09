#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#define MESSAGE_LEN 19
#define DATA_LEN 17
#define MAP_LEN 20
#include "main.h"


typedef struct
{
	
	float Pitch;
	float X;
	float Y;
	float Yaw;
	uint8_t Shot;
	
}possi_buff_typedef;

typedef struct
{
	float X;
	float Y;
	float Yaw;
}map_buff_typedef;

extern possi_buff_typedef PossiBuffRcf;
extern possi_buff_typedef PossiBuffSnd;
extern uint8_t uart1_rx_buffer[MAP_LEN];
extern uint8_t uart1_tx_buffer[MAP_LEN];
extern uint8_t uart6_rx_buffer[MESSAGE_LEN];
extern uint8_t uart6_tx_buffer[MESSAGE_LEN];

void uart1_init(void);
void uart6_init(void);

int Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes);

void Snd_code(possi_buff_typedef *PossiBuffSnd, uint8_t *bytes);

void Map_decode(map_buff_typedef *MapBuffRcf, uint8_t *bytes);

#endif
