#ifndef _PID_H
#define _PID_H
#define abs_new(x) ((x)>0? (x):(-(x)))


typedef struct
{
	float Set;           //定义设定值 
	float Actual;        //定义实际值
	float err;           //定义偏差值 
  float err_last;      //定义上一个偏差值
	float kp;						 //P
	float ki;						 //I
	float kd;						 //D
	float integral;      //定义积分值 
	signed short     int output;				 //输出值
	float integralMax;   //积分上限
	float outputMax;		 //输出上限
	float errMax;        //偏差上限
	float errMin;        //偏差下限	
}
PID_Regulator_t;

void PID_Reset_left_ID1(PID_Regulator_t  *pid);
void PID_Reset_right_ID2(PID_Regulator_t  *pid); 
void PID_Reset_right_position(PID_Regulator_t  *pid);
void PID_Reset_left_position(PID_Regulator_t  *pid);
void PID_Calc(PID_Regulator_t *pid);

#endif

