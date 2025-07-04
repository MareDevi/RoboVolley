#ifndef PID_H
#define PID_H

#include "main.h"
#include "can.h"
#include "rob2.h"

#define KP_G 1.0F
#define KI_G 0.0F
#define KD_G 0.0F
#define IOUT_MAX_G 16.0F
#define OUT_MAX_G 20.0F

#define KP_V 0.5F
#define KI_V 0.0F
#define KD_V 0.0F
#define IOUT_MAX_V 25.0F
#define OUT_MAX_V 40.0F


typedef struct 
{ 
    uint16_t ecd; 
    int16_t speed_rpm; 
    int16_t given_current; 
    uint8_t temperate; 
    int16_t last_ecd; 
}                       gimbal_measure_t; 

extern   gimbal_measure_t   motor_gimbal[3];

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
}                       gimbal_PID_typedef;

extern gimbal_PID_typedef PID_g[3], PID_v[3];

void gimbal_pid_init();

double gimbal_v_pid_calc(RobStrite_Motor* motor, gimbal_PID_typedef pid, double set);

double gimbal_v_pid_calc(RobStrite_Motor* motor, gimbal_PID_typedef pid, double set);

void gimbal_angle_control(RobStrite_Motor* motor, gimbal_PID_typedef pid_g, gimbal_PID_typedef pid_v, double set);

#endif // 
