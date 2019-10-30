#include "pid.h"
#include "main.h"

PID_IncTypeDef spid;
PID_IncTypeDef *speed_pid=&spid;

void Speed_PID_Init(int set_speed)
{
	speed_pid->Kp=0.08;
	speed_pid->Ki=0.03;
	speed_pid->Kd=0;
	speed_pid->expect_value=set_speed;
}

float Speed_Adjust(int now_speed)
{
	float speed_inc;
	speed_pid->error=speed_pid->expect_value-now_speed;
	speed_inc=(speed_pid->Kp*speed_pid->error)-(speed_pid->Ki*speed_pid->pre_error)+(speed_pid->Kd*speed_pid->pre_pre_error);
	speed_pid->pre_pre_error=speed_pid->pre_error;
	speed_pid->pre_error=speed_pid->error;
	return speed_inc;
}