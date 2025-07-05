#include "bsp_uart.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t uart1_rx_buffer[50];

void uart_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}	


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart1)
	{
		HAL_UART_Transmit_DMA(&huart1, uart1_rx_buffer, Size);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}
