#include "rob2.h"
#include "gimbal_pid.h"

gimbal_PID_typedef PID_g[3], PID_v[3];

void gimbal_pid_init()
{
	for(int i=0;i<3;i++)
	{
		PID_g[i].kp = KP_G;
		PID_g[i].ki = KI_G;
		PID_g[i].kd = KD_G;
		PID_g[i].out_max = OUT_MAX_G;
		PID_g[i].iout_max = IOUT_MAX_G;
		PID_g[i].pout = 0;
		PID_g[i].iout = 0;
		PID_g[i].dout = 0;
		PID_g[i].out = 0;
		
		PID_v[i].kp = KP_V;
		PID_v[i].ki = KI_V;
		PID_v[i].kd = KD_V;
		PID_v[i].out_max = OUT_MAX_V;
		PID_v[i].iout_max = IOUT_MAX_V;
		PID_v[i].pout = 0;
		PID_v[i].iout = 0;
		PID_v[i].dout = 0;
		PID_v[i].out = 0;
	}
}

double gimbal_g_pid_calc(RobStrite_Motor* motor, gimbal_PID_typedef pid, double set)
{
	
	
	double target_val = set;
	double get_val = motor->Pos_Info.Angle;
	
	pid.his_error = pid.cur_error;
	pid.cur_error = target_val-get_val;
	pid.pout = KP_G*pid.cur_error;
	pid.iout += KI_G*pid.cur_error;
	pid.dout = KD_G*(pid.cur_error - pid.his_error);
	pid.iout = ((pid.iout > IOUT_MAX_G)? IOUT_MAX_G : pid.iout);
	pid.iout = ((pid.iout < -IOUT_MAX_G)? -IOUT_MAX_G : pid.iout);
	pid.out = pid.pout + pid.iout + pid.dout;
	pid.out = ((pid.out > OUT_MAX_G)? OUT_MAX_G : pid.out);
	pid.out = ((pid.out < -OUT_MAX_G)? -OUT_MAX_G : pid.out);

	return pid.out;
}

double gimbal_v_pid_calc(RobStrite_Motor* motor, gimbal_PID_typedef pid, double set)
{
	double target_val = set;
	double get_val = motor->Pos_Info.Speed;
	
	pid.his_error = pid.cur_error;
	pid.cur_error = target_val-get_val;
	pid.pout = KP_G*pid.cur_error;
	pid.iout += KI_G*pid.cur_error;
	pid.dout = KD_G*(pid.cur_error - pid.his_error);
	pid.iout = ((pid.iout > IOUT_MAX_G)? IOUT_MAX_G : pid.iout);
	pid.iout = ((pid.iout < -IOUT_MAX_G)? -IOUT_MAX_G : pid.iout);
	pid.out = pid.pout + pid.iout + pid.dout;
	pid.out = ((pid.out > OUT_MAX_G)? OUT_MAX_G : pid.out);
	pid.out = ((pid.out < -OUT_MAX_G)? -OUT_MAX_G : pid.out);

	return pid.out;
}

void gimbal_angle_control(RobStrite_Motor* motor, gimbal_PID_typedef pid_g, gimbal_PID_typedef pid_v, double set)
{
	float set_angle = set;
	float set_speed = gimbal_g_pid_calc(motor, pid_g, set_angle);
	float set_current = gimbal_v_pid_calc(motor, pid_v, set_speed);
	
	RobStrite_Motor_current_control(motor, set_current);
}