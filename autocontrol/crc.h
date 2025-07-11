#ifndef CRC_H
#define CRC_H
#include <stdint.h>
extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_TAB[256];
extern const uint16_t CRC_INIT;
extern const uint16_t wCRC_Table[256];
extern uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint8_t ucCRC8);
extern uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
extern uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) ;
#endif