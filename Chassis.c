#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "Task_Init.h"
#include "semphr.h"

 //电机驱动
VESC_t steering1={
	.motor_id=0x01,
	.hcan = &hcan2,
};
VESC_t steering2={ 
	.motor_id=0x02,
	.hcan = &hcan2,
};
VESC_t steering3={
	.motor_id=0x03,
	.hcan = &hcan2,   
};
VESC_t steering4={
	.motor_id=0x04,
	.hcan = &hcan2,
};
PID2 vesc1 = {
	.Kp = 0.95f,
	.Ki = 0.0001f,
	.Kd = 4.22f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
};
PID2 vesc2 = {
	.Kp = 0.95f,
	.Ki = 0.0001f,
	.Kd = 4.22f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
};
PID2 vesc3 = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
};
PID2 vesc4 = {
	.Kp = 0.95f,
	.Ki = 0.0001f,
	.Kd = 4.22f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
};

//遥控模式
Positon_label MODE = REMOTE;

float Vx =0;   //前后移动
float Vy =0;   //左右移动
float Wz =0;   //顺逆自转

//该变量的值可能会被程序外的因素（如硬件、其他线程）修改
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;
volatile float v4 = 0.0f;

volatile float wheel_one = 0.0f;  //前左
volatile float wheel_two = 0.0f;  //前右
volatile float wheel_three=0.0f;  //后右
volatile float wheel_four =0.0f;  //后左

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	  portTickType xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
//		float k = 0.70710678f; //sqrt(2)/2

		if(MODE == REMOTE)
		{			
			
			v1 = -(sqrt(2.0f)/2.0)*(Vx + Vy + 2*LENGTH*Wz);
			v2 = (sqrt(2.0f)/2.0)*(Vx - Vy - 2*LENGTH*Wz);
			vTaskDelay(1);
			v3 = -(sqrt(2.0f)/2.0)*(Vx + Vy - 2*LENGTH*Wz);
			v4 = (sqrt(2.0f)/2.0)*(-Vx + Vy - 2*LENGTH*Wz);
			
//			wheel_one=  (int16_t)((v1 / (2.0f * PI * WHEEL_RADIUS)));
//			wheel_two=  (int16_t)((v2 / (2.0f * PI * WHEEL_RADIUS)));
//			wheel_three=-(int16_t)((v3 / (2.0f * PI * WHEEL_RADIUS)));
//			wheel_four =(int16_t)((v4 / (2.0f * PI * WHEEL_RADIUS)));
			
//			PID_Control2((float)(steering1.epm / 7.0f/(3.4f)), wheel_one, &vesc1);
//			PID_Control2((float)(steering2.epm / 7.0f/(3.4f)), wheel_two, &vesc2);
//			PID_Control2((float)(steering3.epm / 7.0f/(3.4f)), wheel_three,&vesc3);
//			PID_Control2((float)(steering4.epm / 7.0f/(3.4f)), wheel_four, &vesc4);

//        VESC_SetCurrent(&steering1, vesc1.pid_out);
//        VESC_SetCurrent(&steering2, vesc2.pid_out);
//	    vTaskDelay(1);
//	    VESC_SetCurrent(&steering3, vesc3.pid_out);  
//        VESC_SetCurrent(&steering4, vesc4.pid_out);  
		}
		if(MODE == STP || MODE == STOP)
		{
			wheel_one = 0;
			wheel_two = 0;
			wheel_three = 0;
			wheel_four = 0;
			
//			VESC_SetCurrent(&steering1, 0);
//			VESC_SetCurrent(&steering2, 0);
//			vTaskDelay(1);
//			VESC_SetCurrent(&steering3, 0);
//			VESC_SetCurrent(&steering4, 0);
		}
		
		vTaskDelayUntil(&xLastWakeTime,2);
	}
}

extern SemaphoreHandle_t remote_semaphore;

TaskHandle_t Move_Remote_Handle;
void Move_Remote(void *pvParameters){
	
	TickType_t last_wake_time = xTaskGetTickCount();

    for(;;)
    {
        if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            memcpy(&RemoteData, usart4_dma_buff, sizeof(RemoteData));
            Updatekey(&Remote_Control);
            Remote_Control.Ex =-RemoteData.rocker[1];
            Remote_Control.Ey = RemoteData.rocker[0];
            Remote_Control.Eomega = RemoteData.rocker[2];
            Remote_Control.mode = RemoteData.rocker[3];
            Remote_Control.Key_Control = &RemoteData.Key;
        }else{
            Remote_Control.Ex = 0;
            Remote_Control.Ey = 0;
            Remote_Control.Eomega = 0;
            Remote_Control.mode = 0;
            //按键状态清零
            memset(&RemoteData.Key, 0, sizeof(hw_key_t));
            Remote_Control.Key_Control = &RemoteData.Key;
        }

     if(MODE == REMOTE)
        {
					//遥控映射
            Vx = (Remote_Control.Ex / 2047.0f) * MAX_VELOCITY;
            Vy = -(Remote_Control.Ey / 2047.0f) * MAX_VELOCITY;
            Wz = 3*(Remote_Control.Eomega / 2047.0f) * MAX_OMEGA;
        }
//				vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t Recv[8] = {0};
//	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
//	VESC_ReceiveHandler(&steering1, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering2, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering3, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering4, &hcan2, ID,Recv);

//    }
