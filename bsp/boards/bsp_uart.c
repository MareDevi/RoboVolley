#include "bsp_uart.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
//由于不知道传什么信息，发送是基于全场定位 Pitch X Y Yaw 顺序写的。
uint8_t uart1_rx_buffer[33] =
	{0xa5,0x0d,0x00,0x00,0xd3,0x02,0x04
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x00,0x98,0x1f};
uint8_t uart1_tx_buffer[33] =
	{0xa5,0x0d,0x00,0x00,0xd3,0x02,0x04
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x00,0x98,0x1f};

possi_buff_typedef PossiBuffRcf;
possi_buff_typedef PossiBuffSnd;

void uart1_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}	

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
{
		if(huart == &huart1)
	{
		Rcf_decode(&PossiBuffRcf, uart1_rx_buffer);
		Snd_code(&PossiBuffSnd, uart1_tx_buffer);
		HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, Size);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

//-------------------------decode recerive buff------
void Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes) {
    union {
        uint8_t val[4];
        float fval;
    }Pitch;
		union {
        uint8_t val[4];
        float fval;
    }X;
		union {
        uint8_t val[4];
        float fval;
    }Y;
		union {
        uint8_t val[4];
        float fval;
    }Yaw;
    Pitch.val[0] = bytes[8];
    Pitch.val[1] = bytes[9];
    Pitch.val[2] = bytes[10];
    Pitch.val[3] = bytes[11];
    X.val[0] = bytes[12];
    X.val[1] = bytes[13];
    X.val[2] = bytes[14];
    X.val[3] = bytes[15];
		Y.val[0] = bytes[16];
    Y.val[1] = bytes[17];
    Y.val[2] = bytes[18];
    Y.val[3] = bytes[19];
		Yaw.val[0] = bytes[20];
    Yaw.val[1] = bytes[21];
    Yaw.val[2] = bytes[22];
    Yaw.val[3] = bytes[23];

		PossiBuffRcf->Pitch = Pitch.fval;
		PossiBuffRcf->Yaw = Yaw.fval;
		PossiBuffRcf->X = X.fval;
		PossiBuffRcf->Y = Y.fval;
}

//-------------------------code buff to send -------
void Snd_code(possi_buff_typedef *PossiBuffSnd, uint8_t *bytes)
{
	  union {
        float fval;
        uint8_t val[4];
    }Pitch;
		union {
        float fval;
        uint8_t val[4]; 
    }X;
		union {
         float fval;
         uint8_t val[4];
    }Y;
		union {
        float fval;
        uint8_t val[4];
    }Yaw;
		
		Pitch.fval = PossiBuffSnd->Pitch;
		Yaw.fval = PossiBuffSnd->Yaw;
		X.fval = PossiBuffSnd->X;
		Y.fval = PossiBuffSnd->Y;
		
    bytes[8] = Pitch.val[3];
    bytes[9] = Pitch.val[2];
    bytes[10] = Pitch.val[1];
    bytes[11] = Pitch.val[0];
    bytes[12] = X.val[3];
    bytes[13] = X.val[2];
    bytes[14] = X.val[1];
    bytes[15] = X.val[0];
		bytes[16] = Y.val[3];
    bytes[17] = Y.val[2];
    bytes[18] = Y.val[1];
    bytes[19] = Y.val[0];
		bytes[20] = Yaw.val[3];
    bytes[21] = Yaw.val[2];
    bytes[22] = Yaw.val[1];
    bytes[23] = Yaw.val[0];
}
