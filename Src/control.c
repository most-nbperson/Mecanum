#include "control.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"

void Motor_Init()
{
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	TIM5->CCR1=0;	
	TIM5->CCR2=0;
	TIM5->CCR3=0;
	TIM5->CCR4=0;
}

void Control(uint8_t motors,uint16_t speed,uint8_t direction)
{
	if(motors==LF)
	{
		if(direction==FRONT)
		{
			HAL_GPIO_WritePin(LF_IN1_GPIO_Port,LF_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LF_IN2_GPIO_Port,LF_IN2_Pin,GPIO_PIN_SET);
		}
		else if(direction==BACK)
		{
			HAL_GPIO_WritePin(LF_IN1_GPIO_Port,LF_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LF_IN2_GPIO_Port,LF_IN2_Pin,GPIO_PIN_RESET);
		}
		TIM5->CCR2=speed;
	}
	else if(motors==LB)
	{
		if(direction==FRONT)
		{
			HAL_GPIO_WritePin(LB_IN1_GPIO_Port,LB_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LB_IN2_GPIO_Port,LB_IN2_Pin,GPIO_PIN_SET);
		}
		else if(direction==BACK)
		{
			HAL_GPIO_WritePin(LB_IN1_GPIO_Port,LB_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LB_IN2_GPIO_Port,LB_IN2_Pin,GPIO_PIN_RESET);
		}
		TIM5->CCR4=speed;
	}
	else if(motors==RF)
	{
		if(direction==FRONT)
		{
			HAL_GPIO_WritePin(RF_IN1_GPIO_Port,RF_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(RF_IN2_GPIO_Port,RF_IN2_Pin,GPIO_PIN_RESET);
		}
		else if(direction==BACK)
		{
			HAL_GPIO_WritePin(RF_IN1_GPIO_Port,RF_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF_IN2_GPIO_Port,RF_IN2_Pin,GPIO_PIN_SET);
		}
		TIM5->CCR1=speed;
	}
	else if(motors==RB)
	{
		if(direction==FRONT)
		{
			HAL_GPIO_WritePin(RB_IN1_GPIO_Port,RB_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(RB_IN2_GPIO_Port,RB_IN2_Pin,GPIO_PIN_RESET);
		}
		else if(direction==BACK)
		{
			HAL_GPIO_WritePin(RB_IN1_GPIO_Port,RB_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RB_IN2_GPIO_Port,RB_IN2_Pin,GPIO_PIN_SET);
		}
		TIM5->CCR3=speed;
	}
}

void Move(uint16_t LF_speed,uint16_t LB_speed,uint16_t RF_speed,uint16_t RB_speed,uint8_t direction)
{
	if(direction==FRONT)
	{
		Control(LF,LF_speed,FRONT);
		Control(LB,LB_speed,FRONT);
		Control(RF,RF_speed,FRONT);
		Control(RB,RB_speed,FRONT);
	}
	else if(direction==BACK)
	{
		Control(LF,LF_speed,BACK);
		Control(LB,LB_speed,BACK);
		Control(RF,RF_speed,BACK);
		Control(RB,RB_speed,BACK);
	}
	
	
	
	////////////////////////////////////////
	else if(direction==LEFT)
	{
		Control(LF,LF_speed,BACK);
		Control(LB,LB_speed,FRONT);
		Control(RF,RF_speed,FRONT);
		Control(RB,RB_speed,BACK);
	}
	else if(direction==RIGHT)
	{
		Control(LF,LF_speed,FRONT);
		Control(LB,LB_speed,BACK);
		Control(RF,RF_speed,BACK);
		Control(RB,RB_speed,FRONT);
	}
	
//	else if(direction==LEFT_FRONT)
//	{
//		Control(LF,LF_speed,FRONT);
//		Control(LB,LB_speed,FRONT);
//		Control(RF,RF_speed,FRONT);
//		Control(RB,RB_speed,FRONT);
//	}
//	else if(direction==LEFT_BACK)
//	{
//		Control(LF,LF_speed,FRONT);
//		Control(LB,LB_speed,FRONT);
//		Control(RF,RF_speed,FRONT);
//		Control(RB,RB_speed,FRONT);
//	}
//	else if(direction==RIGHT_BACK)
//	{
//		Control(LF,LF_speed,FRONT);
//		Control(LB,LB_speed,FRONT);
//		Control(RF,RF_speed,FRONT);
//		Control(RB,RB_speed,FRONT);
//	}
//	else if(direction==RIGHT_FRONT)
//	{
//		Control(LF,LF_speed,FRONT);
//		Control(LB,LB_speed,FRONT);
//		Control(RF,RF_speed,FRONT);
//		Control(RB,RB_speed,FRONT);
//	}
	/////////////////////////////////
	
	else if(direction==TURN_LEFT)
	{
		Control(LF,LF_speed,BACK);
		Control(LB,LB_speed,BACK);
		Control(RF,RF_speed,FRONT);
		Control(RB,RB_speed,FRONT);
	}
	
	else if(direction==TURN_RIGHT)
	{
		Control(LF,LF_speed,FRONT);
		Control(LB,LB_speed,FRONT);
		Control(RF,RF_speed,BACK);
		Control(RB,RB_speed,BACK);
	}
	
	else if(direction==STOP)
	{
		HAL_GPIO_WritePin(LF_IN1_GPIO_Port,LF_IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LF_IN2_GPIO_Port,LF_IN2_Pin,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(LB_IN1_GPIO_Port,LB_IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LB_IN2_GPIO_Port,LB_IN2_Pin,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(RF_IN1_GPIO_Port,RF_IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF_IN2_GPIO_Port,RF_IN2_Pin,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(RB_IN1_GPIO_Port,RB_IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RB_IN2_GPIO_Port,RB_IN2_Pin,GPIO_PIN_RESET);
	}
}


void Auto_Control(uint16_t L_speed,uint16_t R_speed)
{
	if(L_speed<=0&&R_speed>0)
	{
		Control(LF,R_speed,BACK);
		Control(LB,R_speed,BACK);
		Control(RF,R_speed,FRONT);
		Control(RB,R_speed,FRONT);
	}
	else if(L_speed>0&&R_speed<=0)
	{
		Control(LF,L_speed,FRONT);
		Control(LB,L_speed,FRONT);
		Control(RF,L_speed,BACK);
		Control(RB,L_speed,BACK);
	}
	else
	{
		Control(LF,L_speed,FRONT);
		Control(LB,L_speed,FRONT);
		Control(RF,R_speed,FRONT);
		Control(RB,R_speed,FRONT);
	}
}