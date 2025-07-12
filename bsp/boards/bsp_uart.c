#include "bsp_uart.h"
#include "usart.h"
#include "crc.h"
#include "bsp_buzzer.h"
#include "hchassis.h"
#include <stdio.h>
#include <string.h>
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
uint8_t nucinfo_rx_buf[2][NUCINFO_RX_BUF_NUM];  // 双缓冲区数组
uint8_t uart1_rx_buffer[MAP_LEN];
uint8_t uart6_rx_buffer[MESSAGE_LEN] =
	{0x01,0x09,           // 帧头
	 0x00,                // 击球 
	 0x00,0x00,0x00,0x00, // x
	 0x00,0x00,0x00,0x00, // y
	 0x00,0x00,0x00,0x00, // yaw
	 0x00,0x00,0x00,0x00};// pitch
uint8_t uart6_tx_buffer[MESSAGE_LEN] =
	{0x01,0x09,
	 0x00,
	 0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00};
possi_buff_typedef PossiBuffRcf;
possi_buff_typedef PossiBuffSnd;
//map_buff_typedef MapBuffRcf;
extern uart_packet_t packet;	 
void uart1_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}	
void uart6_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buffer, sizeof(uart6_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}	

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
{
	if(huart == &huart6)
	{
		int receive_error = Rcf_decode(&PossiBuffRcf, uart6_rx_buffer);
		Snd_code(&PossiBuffSnd, uart6_tx_buffer);
		HAL_UART_Transmit_DMA(&huart6, uart6_tx_buffer, Size);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buffer, sizeof(uart6_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	}
	else if(huart == &huart1)
	{
		//Map_decode(&MapBuffRcf, uart1_rx_buffer);
		uart_decode_packet(uart1_rx_buffer,20,&packet);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

//-------------------------decode recerive buff------

void float_to_be_bytes(float value, uint8_t bytes[4]) {
    union {
        float fval;
        uint8_t bval[4];
    } converter;
    
    converter.fval = value;
    // 转换为大端序 (Big-Endian)
    bytes[0] = converter.bval[3];
    bytes[1] = converter.bval[2];
    bytes[2] = converter.bval[1];
    bytes[3] = converter.bval[0];
}
float be_bytes_to_float(const uint8_t bytes[4]) {
    union {
        float fval;
        uint8_t bval[4];
    } converter;
    
    // 大端序转换 (Big-Endian to host)
    converter.bval[3] = bytes[0];
    converter.bval[2] = bytes[1];
    converter.bval[1] = bytes[2];
    converter.bval[0] = bytes[3];
    
    return converter.fval;
}

/**
 * @brief 解码带校验的数据包
 * @param PossiBuffRcf 目标数据结构指针
 * @param bytes 原始数据缓冲区
 * @param data_length 数据包总长度
 * @return 错误状态码:
 *        0 = 成功
 *       -1 = 无效输入参数
 */
int Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes) {
    if (!PossiBuffRcf || !bytes) return -1;
    
    // 帧头校验（前2字节）
    if (bytes[0] != 0x01 || bytes[1] != 0x09) {
        return -1;
    }
    
    // 从字节数组中提取浮点数值
		PossiBuffRcf->Shot    = bytes[2];
    PossiBuffRcf->X       = be_bytes_to_float(&bytes[3]);
    PossiBuffRcf->Y       = be_bytes_to_float(&bytes[7]);
    PossiBuffRcf->Yaw     = be_bytes_to_float(&bytes[11]);
    PossiBuffRcf->Pitch   = be_bytes_to_float(&bytes[15]);
		return 0;
}

//-------------------------code buff to send -------
void Snd_code(possi_buff_typedef *PossiBuffSnd, uint8_t *bytes) {
    uint8_t pitch_bytes[4];
    uint8_t yaw_bytes[4];
    uint8_t x_bytes[4];
    uint8_t y_bytes[4];
		uint8_t shot_bytes;
    
    // 转换浮点数为大端序字节数组
    float_to_be_bytes(PossiBuffSnd->Pitch, pitch_bytes);
    float_to_be_bytes(PossiBuffSnd->Yaw, yaw_bytes);
    float_to_be_bytes(PossiBuffSnd->X, x_bytes);
    float_to_be_bytes(PossiBuffSnd->Y, y_bytes);
		shot_bytes = PossiBuffSnd->Shot;
    
		bytes[0] = 0x01;
		bytes[1] = 0x09;
    
    // 批量设置浮点数字节
    bytes[2] = shot_bytes;
    memcpy(&bytes[3], x_bytes, 4);
    memcpy(&bytes[7], y_bytes, 4);
    memcpy(&bytes[11], yaw_bytes, 4);
		memcpy(&bytes[15], pitch_bytes, 4);
}
Position position;
int uart_decode_packet(uint8_t *buf, uint16_t len, uart_packet_t *packet) {
    if (len < UART_HEADER_LEN + 1 + UART_CRC16_LEN) return -1; // 长度不足

    // 1. 检查SOF
    if (buf[0] != 0xA5 || buf[1] != 0x5A) return -2;

    // 2. 检查length
    uint8_t data_len = buf[2];
    if (len != UART_HEADER_LEN + 1 + data_len + UART_CRC16_LEN) return -3;

    // 3. 检查header crc8
    if (Verify_CRC8_Check_Sum(buf, UART_HEADER_LEN) == 0) return -4;

    // 4. 检查crc16
    if (Verify_CRC16_Check_Sum(buf, len) == 0) return -5;

    // 5. 提取cmd_id和data
    packet->cmd_id = buf[UART_HEADER_LEN];
    packet->data = (uint8_t *)&buf[UART_HEADER_LEN + 1];
    packet->data_len = data_len;
	  //printf("decode_success\r\n");
		memcpy(&position.world_x, &packet->data[0], sizeof(float));
		memcpy(&position.world_y, &packet->data[4], sizeof(float));
		memcpy(&position.world_yaw, &packet->data[8], sizeof(float));
		PossiBuffSnd.X   = position.world_x;
		PossiBuffSnd.Y   = position.world_y;
		//PossiBuffSnd.Yaw = position.world_yaw;
		//chassis_can_cmd(position.world_x,position.world_y,position.world_yaw);
	  //printf("positon:%.2f,%.2f,%.2f\r\n",position.world_x,position.world_y,position.world_yaw);
	  //HAL_UART_Transmit(&huart6,position.world_x,sizeof(position.world_x),100);
	  //HAL_UART_Transmit(&huart6,position.world_x,sizeof(position.world_x),100);
    return 0; // 成功
}
