#ifndef _PID_H_
#define _PID_H_

typedef struct
{
  float Kp;                       //����ϵ��Proportional
  float Ki;                       //����ϵ��Integral
  float Kd;                       //΢��ϵ��Derivative
 
  float error;               			//��ǰ���
  float pre_error;           			//ǰһ����� e(k-1)
  float pre_pre_error;        		//��ǰһ����� e(k-2)
  float LocSum;                   //�ۼƻ���λ��
}PID_LocTypeDef;


typedef struct
{
  float Kp;                       //����ϵ��Proportional
  float Ki;                       //����ϵ��Integral
  float Kd;                       //΢��ϵ��Derivative
 
  float error;              			//��ǰ���
  float pre_error;         				//ǰһ����� e(k-1)
  float pre_pre_error;         		//��ǰһ����� e(k-2)
	
	int expect_value;
}PID_IncTypeDef;

void Speed_PID_Init(int set_speed);
void Turn_PID_Init();
float Speed_Adjust(int now_speed);
float Turn_Speed_Add(int angle);



#endif