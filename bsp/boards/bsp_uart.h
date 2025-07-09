#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#define MESSAGE_LEN 19
#define DATA_LEN 17
#define MAP_LEN 20
#define SOF1 0xA5
#define SOF2 0x5A
#define NUCINFO_RX_BUF_NUM 46u
#define NUCINFO_FRAME_LENGTH 20u
#define UART_HEADER_LEN 5
#define UART_CRC16_LEN 2
#include "main.h"
#include "struct_typedef.h"
extern uint8_t nucinfo_rx_buf[2][NUCINFO_RX_BUF_NUM];
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




typedef struct {
    float world_x;
    float world_y;
    float world_yaw;
		float self_x;
		float self_y;
}Position;  

typedef struct {
    uint8_t cmd_id;           // 命令ID
    uint8_t *data;            // 数据指针
    uint8_t data_len;         // 数据长度
} uart_packet_t;
int uart_decode_packet(uint8_t *buf, uint16_t len, uart_packet_t *packet);

#endif
