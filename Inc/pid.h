#ifndef _PID_H_
#define _PID_H_

typedef struct
{
  float Kp;                       //比例系数Proportional
  float Ki;                       //积分系数Integral
  float Kd;                       //微分系数Derivative
 
  float error;               			//当前误差
  float pre_error;           			//前一次误差 e(k-1)
  float pre_pre_error;        		//再前一次误差 e(k-2)
  float LocSum;                   //累计积分位置
}PID_LocTypeDef;


typedef struct
{
  float Kp;                       //比例系数Proportional
  float Ki;                       //积分系数Integral
  float Kd;                       //微分系数Derivative
 
  float error;              			//当前误差
  float pre_error;         				//前一次误差 e(k-1)
  float pre_pre_error;         		//再前一次误差 e(k-2)
	
	int expect_value;
}PID_IncTypeDef;

void Speed_PID_Init(int set_speed);
void Turn_PID_Init();
float Speed_Adjust(int now_speed);
float Turn_Speed_Add(int angle);



#endif