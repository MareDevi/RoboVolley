#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
#include "tim.h"

typedef enum {
    BUZZER_STATE_IDLE,      // 初始状态
    BUZZER_STATE_FIRST_UP,  // 已检测到第一次 SW_UP (sw[1]==3)
    BUZZER_STATE_MID,       // 已检测到 SW_MID (sw[1]==1)
    BUZZER_STATE_TRIGGERED  // 完成序列，触发蜂鸣器
} BuzzerState;

extern void buzzer_on();
extern void buzzer_off(void);

#endif
