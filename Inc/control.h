#ifndef __CONTROL_H
#define	__CONTROL_H

#include "main.h"

enum Motors
{
	LF,LB,RB,RF
};

enum Directions
{
	FRONT,BACK,LEFT,RIGHT,
	LEFT_FRONT,LEFT_BACK,RIGHT_FRONT,RIGHT_BACK,
	TURN_RIGHT,TURN_LEFT
};

void Motor_Init();
void Control(uint8_t motors,uint16_t speed,uint8_t direction);
void Move(uint16_t LF_speed,uint16_t LB_speed,uint16_t RF_speed,uint16_t RB_speed,uint8_t direction);


#endif