#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"


typedef struct
{
	
	float Pitch;
	float X;
	float Y;
	float Yaw;
	
}possi_buff_typedef;

extern possi_buff_typedef PossiBuffRcf;
extern possi_buff_typedef PossiBuffSnd;
extern uint8_t uart1_rx_buffer[33];
extern uint8_t uart1_tx_buffer[33];

void uart1_init(void);

void Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes);

void Snd_code(possi_buff_typedef *PossiBuffSnd, uint8_t *bytes);


#endif
