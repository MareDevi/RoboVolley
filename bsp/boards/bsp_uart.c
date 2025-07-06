#include "bsp_uart.h"
#include "usart.h"
#include "crc.h"
#include "bsp_buzzer.h"
extern DMA_HandleTypeDef hdma_usart1_rx;
//由于不知道传什么信息，发送是基于全场定位 Pitch X Y Yaw 顺序写的。
uint8_t uart1_rx_buffer[25] =
	{0xa5,0x10,0x00,0x00,0xd3,0x00,0x00
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x98,0x1f};
uint8_t uart1_tx_buffer[25] =
	{0xa5,0x10,0x00,0x00,0xd3,0x00,0x00
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x98,0x1f};
//0~6为固定位 7~10为pitch 11~14为X 15~18为Y 19~22为yaw 23~24为校验位
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
		int receive_error = Rcf_decode(&PossiBuffRcf, uart1_rx_buffer);
		Snd_code(&PossiBuffSnd, uart1_tx_buffer);
		HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, Size);
		
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
 *       -2 = CRC8 校验失败
 *       -3 = CRC16 校验失败
 */
int Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes) {
    if (!PossiBuffRcf || !bytes) return -1;
    
    // CRC8 校验头（前4字节）
    if (bytes[4] != Get_CRC8_Check_Sum(bytes, 4, CRC8_INIT)) {
        return -2;
    }
    
    // 使用现有函数进行CRC16校验
    uint16_t crc_calc = Get_CRC16_Check_Sum(bytes, 23, CRC_INIT);
    uint16_t crc_recv = (bytes[23] << 8) | bytes[24];
    
    if (crc_calc != crc_recv) {
        return -3;
    }
    
    // 从字节数组中提取浮点数值
    PossiBuffRcf->Pitch = be_bytes_to_float(&bytes[7]);
    PossiBuffRcf->X     = be_bytes_to_float(&bytes[11]);
    PossiBuffRcf->Y     = be_bytes_to_float(&bytes[15]);
    PossiBuffRcf->Yaw   = be_bytes_to_float(&bytes[19]);

}

//-------------------------code buff to send -------
void Snd_code(possi_buff_typedef *PossiBuffSnd, uint8_t *bytes) {
    uint8_t pitch_bytes[4];
    uint8_t yaw_bytes[4];
    uint8_t x_bytes[4];
    uint8_t y_bytes[4];
    
    // 转换浮点数为大端序字节数组
    float_to_be_bytes(PossiBuffSnd->Pitch, pitch_bytes);
    float_to_be_bytes(PossiBuffSnd->Yaw, yaw_bytes);
    float_to_be_bytes(PossiBuffSnd->X, x_bytes);
    float_to_be_bytes(PossiBuffSnd->Y, y_bytes);
    
    // 设置初始CRC校验
    bytes[4] = Get_CRC8_Check_Sum(bytes, 4, CRC8_INIT);
    
    // 批量设置浮点数字节
    memcpy(&bytes[7], pitch_bytes, 4);
    memcpy(&bytes[11], x_bytes, 4);
    memcpy(&bytes[15], y_bytes, 4);
    memcpy(&bytes[19], yaw_bytes, 4);
    
    // 计算并设置CRC16校验
    uint16_t crc16 = Get_CRC16_Check_Sum(bytes, 23, CRC_INIT);
    bytes[23] = (crc16 >> 8) & 0xFF;  // 存储高字节
    bytes[24] = crc16 & 0xFF;         // 存储低字节
}
