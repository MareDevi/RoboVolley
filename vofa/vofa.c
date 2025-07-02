#include "vofa.h"
#include "usart.h"
#include "stm32f4xx.h"                  // Device header
#include "RTE_Components.h"             // Component selection
#include "stdio.h"

int fputc(int ch, FILE *f)
 
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
}
