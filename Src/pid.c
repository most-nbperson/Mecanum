#include "pid.h"
#include "main.h"
#include <math.h>

PID_IncTypeDef spid;
PID_IncTypeDef *speed_pid=&spid;//用于控制速度

PID_IncTypeDef tpid;//用于差速转弯
PID_IncTypeDef *turn_pid=&tpid;


void Speed_PID_Init(int set_speed)
{
	speed_pid->Kp=0.08;
	speed_pid->Ki=0.03;
	speed_pid->Kd=0;
	speed_pid->expect_value=set_speed;
}

void Turn_PID_Init()
{
	turn_pid->Kp=0.09;
	turn_pid->Ki=0;
	turn_pid->Kd=0;
	turn_pid->expect_value=0;
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

float Turn_Speed_Add(int angle)
{
	if(angle<0)
		angle=-angle;
	
	float pwm_inc;
	turn_pid->error=angle;
	pwm_inc=(turn_pid->Kp*turn_pid->error)-(turn_pid->Ki*turn_pid->pre_error)+(turn_pid->Kd*turn_pid->pre_pre_error);
	turn_pid->pre_pre_error=turn_pid->pre_error;
	turn_pid->pre_error=turn_pid->error;
	return pwm_inc;
}

