
#include "main.h"
#include "stdio.h"

//void usart_printf(const char *fmt,...)
//{
// static uint8_t tx_buf[256] = {0};
// static va_list ap;
// static uint16_t len;
// va_start(ap, fmt);
// //return length of string 
// //�����ַ�������
// len = vsprintf((char *)tx_buf, fmt, ap);
// va_end(ap);
// usart1_tx_dma_enable(tx_buf, len);
//}
extern UART_HandleTypeDef huart6;


/**
  * @brief  Retargets the C library printf function to the UART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{
	// ������ѯ��ʽ����1�ֽ����ݣ���ʱʱ������Ϊ���޵ȴ� 
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

/**
  * @brief  Retargets the C library scanf function to the UART.
  * @param  None
  * @retval None
  */
int fgetc(FILE *f)
{
	uint8_t ch;
	// ������ѯ��ʽ����1�ֽ����ݣ���ʱʱ������Ϊ���޵ȴ� 
	HAL_UART_Receive( &huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY );
	return ch;;
}