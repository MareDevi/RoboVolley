#include "bsp_uart.h"
#include "usart.h"
#include "crc.h"
#include "bsp_buzzer.h"
extern DMA_HandleTypeDef hdma_usart1_rx;
//���ڲ�֪����ʲô��Ϣ�������ǻ���ȫ����λ Pitch X Y Yaw ˳��д�ġ�
uint8_t uart1_rx_buffer[25] =
	{0xa5,0x10,0x00,0x00,0xd3,0x00,0x00
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x98,0x1f};
uint8_t uart1_tx_buffer[25] =
	{0xa5,0x10,0x00,0x00,0xd3,0x00,0x00
	,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	,0x98,0x1f};
//0~6Ϊ�̶�λ 7~10Ϊpitch 11~14ΪX 15~18ΪY 19~22Ϊyaw 23~24ΪУ��λ
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
    // ת��Ϊ����� (Big-Endian)
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
    
    // �����ת�� (Big-Endian to host)
    converter.bval[3] = bytes[0];
    converter.bval[2] = bytes[1];
    converter.bval[1] = bytes[2];
    converter.bval[0] = bytes[3];
    
    return converter.fval;
}

/**
 * @brief �����У������ݰ�
 * @param PossiBuffRcf Ŀ�����ݽṹָ��
 * @param bytes ԭʼ���ݻ�����
 * @param data_length ���ݰ��ܳ���
 * @return ����״̬��:
 *        0 = �ɹ�
 *       -1 = ��Ч�������
 *       -2 = CRC8 У��ʧ��
 *       -3 = CRC16 У��ʧ��
 */
int Rcf_decode(possi_buff_typedef *PossiBuffRcf, uint8_t *bytes) {
    if (!PossiBuffRcf || !bytes) return -1;
    
    // CRC8 У��ͷ��ǰ4�ֽڣ�
    if (bytes[4] != Get_CRC8_Check_Sum(bytes, 4, CRC8_INIT)) {
        return -2;
    }
    
    // ʹ�����к�������CRC16У��
    uint16_t crc_calc = Get_CRC16_Check_Sum(bytes, 23, CRC_INIT);
    uint16_t crc_recv = (bytes[23] << 8) | bytes[24];
    
    if (crc_calc != crc_recv) {
        return -3;
    }
    
    // ���ֽ���������ȡ������ֵ
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
    
    // ת��������Ϊ������ֽ�����
    float_to_be_bytes(PossiBuffSnd->Pitch, pitch_bytes);
    float_to_be_bytes(PossiBuffSnd->Yaw, yaw_bytes);
    float_to_be_bytes(PossiBuffSnd->X, x_bytes);
    float_to_be_bytes(PossiBuffSnd->Y, y_bytes);
    
    // ���ó�ʼCRCУ��
    bytes[4] = Get_CRC8_Check_Sum(bytes, 4, CRC8_INIT);
    
    // �������ø������ֽ�
    memcpy(&bytes[7], pitch_bytes, 4);
    memcpy(&bytes[11], x_bytes, 4);
    memcpy(&bytes[15], y_bytes, 4);
    memcpy(&bytes[19], yaw_bytes, 4);
    
    // ���㲢����CRC16У��
    uint16_t crc16 = Get_CRC16_Check_Sum(bytes, 23, CRC_INIT);
    bytes[23] = (crc16 >> 8) & 0xFF;  // �洢���ֽ�
    bytes[24] = crc16 & 0xFF;         // �洢���ֽ�
}
