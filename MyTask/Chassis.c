#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "Task_Init.h"
#include "semphr.h"
#include "dataFrame.h"
#include "comm.h"
#include "comm_stm32_hal_middle.h"
#include "data_poll.h"
#include "My_list.h"
#include "math.h"
#include "JY61.h"

extern SemaphoreHandle_t Jy61_semaphore;
extern SemaphoreHandle_t remote_semaphore;

//陀螺仪姿态矫正
PID2 JY61_adjust = {
	.Kp = 0.6f,
	.Ki = 0.0005f,
	.Kd = 0.4f,
	.limit = 10000.0f,
	.output_limit = 50.0f,
};

//遥控器
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
uint8_t usart5_buff[30];
uint8_t uart4_buff[30];
int16_t motorCurrentBuf[4] = {0}; // 用于3508发送变量

//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 20.0f,
	.Ki = 1.5f,
	.Kd = 2.0f,
	.limit = 16384.0f,
	.output_limit = 10000.0f,
},
.steering={
	.ID=0x201,
	.hcan = &hcan2,
}
};
Motor_param motor2 = {
.PID = {
	.Kp = 20.0f,
	.Ki = 1.5f,
	.Kd = 2.0f,
	.limit = 16384.0f,
	.output_limit = 10000.0f,
},
.steering={
	.ID=0x202,
	.hcan = &hcan2,                                                                                                                                                             
}
};
Motor_param motor3 = {
.PID = {
	.Kp = 20.0f,
	.Ki = 1.5f,
	.Kd = 2.0f,
	.limit = 16384.0f,
	.output_limit = 10000.0f,
},
.steering={
	.ID=0x203,
	.hcan = &hcan2,
}
};


//遥控模式
Chassis_label MODE = REMOTE;

volatile float Vx =0;   //前后移动
volatile float Vy =0;   //左右移动
volatile float Wz =0;   //顺逆自转

volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;

volatile float wheel_one = 0.0f;
volatile float wheel_two = 0.0f;
volatile float wheel_three=0.0f;

float lowpass_filter(float new_sample, float *prev_filter, float alpha)
{
    *prev_filter = (1.0f - alpha) * (*prev_filter) + alpha * new_sample;
    return *prev_filter;
}

static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Remote_Analysis()
{
    if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
			
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = - recv_pack.rocker[0] / 1647.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[1] / 1647.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1647.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    xSemaphoreGive(remote_semaphore);
//	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
   		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
	}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

TaskHandle_t Remote_Go_Handle;
void Remote_Go(void *pvParameters)
{
   for(;;)
	{
		Remote_Analysis();
	 }
}

volatile float Wz_correction;//反馈值

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{                                                             
	TickType_t last_wake_time = xTaskGetTickCount();
	
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	for(;;)
	{
		if(MODE == REMOTE)
		{			
			Vx = Remote_Control.Ex;
			Vy = -Remote_Control.Ey;
float Wz_cmd = Remote_Control.Eomega;
			//前馈
			float Wz_ff = 0;
			
			static float Vy_last = 0;
			static float dVy_f = 0;
			float dt = 0.002f;

			float dVy = (Vy - Vy_last) / dt;
			Vy_last = Vy;

			//Vy限幅（根据最大速度4/dt）(他妈的实际上谁能一帧到2000)
			if (dVy > 300.0f)  {dVy = 300.0f;}
			if (dVy < -300.0f) {dVy = -300.0f;}

			//滤波
			dVy_f = lowpass_filter(dVy, &dVy_f, 0.5f);

			if (fabs(dVy_f) > 50.0f)
			{
				Wz_ff = -0.015f * dVy_f;
			}
			else
			{
				Wz_ff = 0;
			}
			//前馈Wz限幅
			if (Wz_ff > 0.314f)  {Wz_ff = 0.314f;}
			if (Wz_ff < -0.314f) {Wz_ff = -0.314f;}
			//矫正判断
			if (fabs(Wz_cmd) >= Deadzone_Z)
			{
				Wz = Wz_cmd;
			}
			else
			{
				float k = fabs(Vy) / (fabs(Vx) + fabs(Vy) + 0.001f);
				if (k > 0.5f) { k = 0.5f;} 
				Wz = Wz_cmd + k * Wz_correction + Wz_ff;
			}
			//最终滤波
			static float Wz_out = 0;
			Wz_out = lowpass_filter(Wz, &Wz_out, 0.6f);

			Wz = Wz_out;
			
			v1 = +Vx*0.5f+Vy*(sqrtf(3.0f)/2.0f) - R * Wz;
			v2 = -Vx*0.5f+Vy*(sqrtf(3.0f)/2.0f) + R * Wz;
			v3 = -Vx - R * Wz;			
			
			wheel_one=  -((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two = (( v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=-((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(((float)motor1.steering.motor.Speed)), wheel_one, &motor1.PID);
      PID_Control2((float)(((float)motor2.steering.motor.Speed)), wheel_two, &motor2.PID);
			PID_Control2((float)(((float)motor3.steering.motor.Speed)), wheel_three, &motor3.PID);
			
			motorCurrentBuf[0] = motor1.PID.pid_out;
			motorCurrentBuf[1] = motor2.PID.pid_out;
			motorCurrentBuf[2] = motor3.PID.pid_out;
			
			MotorSend(&hcan2, 0x200, motorCurrentBuf);
			
		}
		if(MODE == STP || MODE == STOP)
		{
			wheel_one = 0;
			wheel_two = 0;
			wheel_three=0;

			MotorSend(&hcan2, 0x200, 0);
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	}
}

TaskHandle_t Remote_JY61_Handle;
void Remote_JY61(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	
   static float gyro_z_filter = 0;
	 
   for(;;)
	{
	  float gyro_z = JY61.AngularVelocity.Z;
		
    gyro_z_filter = lowpass_filter(gyro_z, &gyro_z_filter, 0.38f);
    gyro_z = gyro_z_filter;
		
		if (fabs(gyro_z) < 0.3f)
		{
			gyro_z = 0;
		}
  
		PID_Control2(-gyro_z, 0.0f, &JY61_adjust);

		float out = JY61_adjust.pid_out;

		if (out > 3.5f)  {out = 3.5f;}
		if (out < -3.5f) {out = -3.5f;}
		
		if (fabs(out) < 0.3f)//防抖
		{
			out = 0;
		}
		//滤波
		static float wz_f = 0;
    wz_f = lowpass_filter(out, &wz_f, 0.7f);

		Wz_correction = wz_f; // 转至Remote任务

	 vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	 }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	if (hcan->Instance == CAN2)
	{
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	Motor3508Recv(&motor1.steering, hcan, ID, Recv);
	Motor3508Recv(&motor2.steering, hcan, ID, Recv);
	Motor3508Recv(&motor3.steering, hcan, ID, Recv);
	}
}

