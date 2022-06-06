#ifndef _PID_H
#define _PID_H
#define abs_new(x) ((x)>0? (x):(-(x)))


typedef struct
{
	float Set;           //�����趨ֵ 
	float Actual;        //����ʵ��ֵ
	float err;           //����ƫ��ֵ 
  float err_last;      //������һ��ƫ��ֵ
	float kp;						 //P
	float ki;						 //I
	float kd;						 //D
	float integral;      //�������ֵ 
	signed short     int output;				 //���ֵ
	float integralMax;   //��������
	float outputMax;		 //�������
	float errMax;        //ƫ������
	float errMin;        //ƫ������	
}
PID_Regulator_t;

void PID_Reset_left_ID1(PID_Regulator_t  *pid);
void PID_Reset_right_ID2(PID_Regulator_t  *pid); 
void PID_Reset_right_position(PID_Regulator_t  *pid);
void PID_Reset_left_position(PID_Regulator_t  *pid);
void PID_Calc(PID_Regulator_t *pid);

#endif

