/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_imu.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern imu_t imu;
PID_Regulator_t left,right,left_pos_struct,right_pos_struct;
CAN_RxHeaderTypeDef canRxHeader;
CAN_TxHeaderTypeDef canTxHeader;
uint8_t canRxCache[8];
uint8_t canTxCache[8];
uint32_t txMailBox = 0;
float time;

struct Feedback
{
	unsigned short Mechanical_angle;
	short Rotor_speed;
	unsigned short Output_torque;
}Electric_control[4];
extern CAN_HandleTypeDef hcan1;
void CAN_Send_Msg(void)
{		
	canTxHeader.StdId =0x200;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = 0x08;

	canTxCache[0]=(left.output>>8)&0xff;
	canTxCache[1]=left.output&0xff;
	canTxCache[2]=(right.output>>8)&0xff;
	canTxCache[3]=right.output&0xff;

	HAL_CAN_AddTxMessage(&hcan1,&canTxHeader,canTxCache, &txMailBox);
}

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Steering_TaskHandle;
osThreadId Debugging_TaskHandle;
osThreadId M2006_TaskHandle;
osThreadId Mpu_TaskHandle;

SemaphoreHandle_t M2006_BinarySem_Handle =NULL;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Steering_DefaultTask(void const * argument);
void Debugging_DefaultTask(void const * argument);
void M2006_DefaultTask(void const * argument);
void Mpu_DefaultTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE; 
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

	M2006_BinarySem_Handle = xSemaphoreCreateBinary();			
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
/*
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
*/
  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Steering_gear_task, Steering_DefaultTask, 7, 0, 128);
  Steering_TaskHandle = osThreadCreate(osThread(Steering_gear_task), NULL);
	
	osThreadDef(Conrol_M2006_task, M2006_DefaultTask, 9, 0, 128);
  M2006_TaskHandle = osThreadCreate(osThread(Conrol_M2006_task), NULL);
	
	osThreadDef(Debugging_task, Debugging_DefaultTask, 6, 0, 128);
  Debugging_TaskHandle = osThreadCreate(osThread(Debugging_task), NULL);
	
	osThreadDef(Mpu_Task, Mpu_DefaultTask, 8, 0, 128);
  Mpu_TaskHandle = osThreadCreate(osThread(Mpu_Task), NULL);
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Steering_DefaultTask(void const * argument)																//舵机
{
  /* USER CODE BEGIN StartDefaultTask */
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  /* Infinite loop */
  for(;;)
  {

		if(pdTRUE == xReturn)
		{
			TIM8->CCR4 = 4490;							//down 4490闭合				up 2000最高										4490闭合--------3500 抬起最佳
//			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET);														//电磁气动阀
//			osDelay(2000);
//			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_RESET);
//			osDelay(2000);
		}
  }
  /* USER CODE END StartDefaultTask */
}
void M2006_DefaultTask(void const * argument)																			//地盘驱动		right 负 正方向，left 正 正方向
{
  /* USER CODE BEGIN StartDefaultTask */
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  /* Infinite loop */
  for(;;)
  {
		//获取二值信号量 xSemaphore,没获取到则一直等待
		xReturn = xSemaphoreTake(M2006_BinarySem_Handle,/* 二值信号量句柄 */
                              portMAX_DELAY); /* 等待时间 */			
		if(pdTRUE == xReturn)
		{	
				if(time<=3)
{
				right_pos_struct.Set=-100;
				left_pos_struct.Set=100;
}
				else
{	
				right_pos_struct.Set=0;
				left_pos_struct.Set=0;
}



				left.Actual=Electric_control[0].Rotor_speed;
				right.Actual=Electric_control[1].Rotor_speed;
			
				if(abs(right_pos_struct.Actual-right_pos_struct.Set)>20)
					right_pos_struct.Actual+=right.Actual/10000;
				else
					right_pos_struct.Actual=0;
				
				if(abs(left_pos_struct.Actual-left_pos_struct.Set)>20)
					left_pos_struct.Actual+=right.Actual/10000;
				else
					left_pos_struct.Actual=0;
				
			
				PID_Calc(&left_pos_struct);
				PID_Calc(&right_pos_struct);
			
				left.Set=left_pos_struct.output;
				right.Set=right_pos_struct.output;
			
				PID_Calc(&left);
				PID_Calc(&right);
			/********************Limiting amplitude******************/
			if(abs(right_pos_struct.Actual-right_pos_struct.Set)<=20)
			{
				right_pos_struct.Set=0;
				right_pos_struct.Actual=0;
				right_pos_struct.output=0;
				right_pos_struct.integral=0;
				right.Set=0;
				right.Actual=0;
				right.output=0;
				right.integral=0;
			}
			if(abs(left_pos_struct.Actual-left_pos_struct.Set)<=20)
			{
				left_pos_struct.Set=0;
				left_pos_struct.Actual=0;
				left_pos_struct.output=0;
				left_pos_struct.integral=0;
				left.Set=0;
				left.Actual=0;
				left.output=0;
				left.integral=0;
			}
			/********************************************************/

				


				CAN_Send_Msg();
		
			}

		}
  }
  /* USER CODE END StartDefaultTask */

void Mpu_DefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	 BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  /* Infinite loop */
  for(;;)
  {

		if(pdTRUE == xReturn)
		{
			
			osDelay(500);
		}
  }
  /* USER CODE END StartDefaultTask */
}
void Debugging_DefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {
		time=time+0.5;
		printf("%f\t%f\t%d\t%d\r\n",right_pos_struct.Set,right_pos_struct.Actual,right_pos_struct.output,right.output); //电机位置设定值，电机输出值	
		printf("%f\t%f\t%d\t%d\r\n",left_pos_struct.Set,left_pos_struct.Actual,left_pos_struct.output,left.output);			//电机位置设定值，电机输出值	
		printf(" Roll: %8.3lf    Pitch: %8.3lf    Yaw: %8.3lf\r\n", imu.rol, imu.pit, imu.yaw);													//陀螺仪打印
		printf("时间：%.3lf\r\n",time);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_FilterFIFO0, &canRxHeader, canRxCache);
	
	switch(canRxHeader.StdId)
	{

			case 0x201:
								Electric_control[0].Mechanical_angle=canRxCache[0];
								Electric_control[0].Mechanical_angle=(Electric_control[0].Mechanical_angle<<8)+canRxCache[1];
								Electric_control[0].Rotor_speed=canRxCache[2];
								Electric_control[0].Rotor_speed=(Electric_control[0].Rotor_speed<<8)+canRxCache[3];
								Electric_control[0].Output_torque=canRxCache[4];
								Electric_control[0].Output_torque=(Electric_control[0].Output_torque<<8)+canRxCache[5];
								break;
			case 0x202:
								Electric_control[1].Mechanical_angle=canRxCache[0];
								Electric_control[1].Mechanical_angle=(Electric_control[1].Mechanical_angle<<8)+canRxCache[1];
								Electric_control[1].Rotor_speed=canRxCache[2];
								Electric_control[1].Rotor_speed=(Electric_control[1].Rotor_speed<<8)+canRxCache[3];
								Electric_control[1].Output_torque=canRxCache[4];
								Electric_control[1].Output_torque=(Electric_control[1].Output_torque<<8)+canRxCache[5];
								break;
			case 0x203:
								Electric_control[2].Mechanical_angle=canRxCache[0];
								Electric_control[2].Mechanical_angle=(Electric_control[2].Mechanical_angle<<8)+canRxCache[1];
								Electric_control[2].Rotor_speed=canRxCache[2];
								Electric_control[2].Rotor_speed=(Electric_control[2].Rotor_speed<<8)+canRxCache[3];
								Electric_control[2].Output_torque=canRxCache[4];
								Electric_control[2].Output_torque=(Electric_control[2].Output_torque<<8)+canRxCache[5];	
								break;
			case 0x204:
								Electric_control[3].Mechanical_angle=canRxCache[0];
								Electric_control[3].Mechanical_angle=(Electric_control[3].Mechanical_angle<<8)+canRxCache[1];
								Electric_control[3].Rotor_speed=canRxCache[2];
								Electric_control[3].Rotor_speed=(Electric_control[3].Rotor_speed<<8)+canRxCache[3];
								Electric_control[3].Output_torque=canRxCache[4];
								Electric_control[3].Output_torque=(Electric_control[3].Output_torque<<8)+canRxCache[5];
								break;
			default:  printf("recive ERROR\r\n"); break;
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
