#ifndef imu_calc_H
#define imu_calc_H

#include "main.h"
#include "can.h"


#define KP 11.5F
#define KI 0.05F
#define KD 0.001F
#define IOUT_MAX 800.0F
#define OUT_MAX 2000.0F


typedef struct 
{ 
    uint16_t ecd; 
    int16_t speed_rpm; 
    int16_t given_current; 
    uint8_t temperate; 
    int16_t last_ecd; 
}                       motor_measure_t; 

extern   motor_measure_t   motor_chassis[3];

typedef struct 
{ 
  double kp;
	double ki;
	double kd;
	double pout;
	double iout;
	double dout;
	double out;
	double iout_max;
	double out_max;
	double cur_error;
	double his_error;
	int test_init;
	int test_start;
}                       PID_typedef;

extern PID_typedef PID1, PID2, PID3;

void can_filter_init(void);

void chassis_can_cmd(int16_t,int16_t,int16_t);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

//void chassis_motor_elec_set(void);

void shut_up(void);

void get_motor_measure(motor_measure_t*, uint8_t*);

void pid_init(PID_typedef);

double chassis_motor_1_pid();

double chassis_motor_2_pid();

double chassis_motor_3_pid();

#endif //