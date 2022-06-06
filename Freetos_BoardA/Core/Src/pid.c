#include  "pid.h"
int right_speed;
int left_speed;
float right_position,left_position;
/*
   pid.c使用说明：此文件代码作为初始化PID 参数以及PID 算法运算用
*/
void PID_Reset_right_ID2(PID_Regulator_t  *pid)    
{
	 pid->Set=0;    
   pid->Actual=0.0;   
   pid->err=0.0;     
   pid->err_last=0.0;       
   pid->integral=0.0;   
	 pid->kp=0.75;//0.75
   pid->ki=0.060;//0.08
   pid->kd=0;
	 pid->output=0;
	 pid->integralMax=2000;
	 pid->outputMax=3000;
}

void PID_Reset_left_ID1(PID_Regulator_t  *pid)
{
	 pid->Set=0;    
   pid->Actual=0.0;   
   pid->err=0.0;     
   pid->err_last=0.0;       
   pid->integral=0.0;   
	 pid->kp=0.75;
   pid->ki=0.060;
   pid->kd=0; 
	 pid->output=0;
	 pid->integralMax=2000;
	 pid->outputMax=3000;
}
void PID_Reset_right_position(PID_Regulator_t  *pid)    
{
	 pid->Set=-right_position;    
   pid->Actual=0.0;   
   pid->err=0.0;     
   pid->err_last=0.0;       
   pid->integral=0.0;   
	 pid->kp=8;
   pid->ki=1.5;
   pid->kd=0;
	 pid->output=0;
	 pid->integralMax=2000;
	 pid->outputMax=4000;
}
void PID_Reset_left_position(PID_Regulator_t  *pid)    
{
	 pid->Set=left_position;    
   pid->Actual=0.0;   
   pid->err=0.0;     
   pid->err_last=0.0;       
   pid->integral=0.0;   
	 pid->kp=8;
   pid->ki=1.5;
   pid->kd=0;
	 pid->output=0;
	 pid->integralMax=2000;
	 pid->outputMax=4000;
}
void PID_Calc(PID_Regulator_t *pid)
{
	 pid->err=pid->Set-pid->Actual;
     pid->integral+=pid->err;
     pid->output=pid->kp*pid->err+pid->ki*pid->integral+pid->kd*(pid->err-pid->err_last); 
     pid->err_last=pid->err;  
	 //限幅
	if(abs_new(pid->output) > pid->outputMax)
	{
			   if(pid->output>0)
			   {
					pid->output=pid->outputMax;
			   }
			   else
			   {
					pid->output=-pid->outputMax;
			   }		   
	}	 
   //限积分量
	if(abs_new(pid->integral) > pid->integralMax)
	{
			   if(pid->integral>0)
			   {
					pid->integral=pid->integralMax;
			   }
			   else
			   {
					pid->integral=-pid->integralMax;
			   }		   
	}	

}

