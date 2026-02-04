#include "PID_old.h"
#include "hitball.h"
#include "Task_Init.h"
#include "RobStride.h"
#include "math.h "
#include "step.h"

CubicParam_t traj_up;
CubicParam_t traj_down;

TrajectoryState_t traj_up_state;
TrajectoryState_t traj_down_state;

RobStride_t R_up;
RobStride_Expect R_up_expect;
//RobStride_Expect R_up_expect = 
//{
//.expect_torque = 0.0f,
//.expect_angle = 0.0f,
//.expect_omega = 0.0f
//};

RobStride_t R_down;
RobStride_Expect R_down_expect;
//RobStride_Expect R_down_expect = 
//{	.expect_torque = 0.0f,
//  .expect_angle = 0.0f,
//  .expect_omega = 0.0f
//};

//R_up_PID R_UP_PID = {
//    .pos_pid = {
//        .Kp = 0.0f,
//        .Ki = 0.0f,
//        .Kd = 0.0f,
//        .limit = 1000.0f,
//        .output_limit = 500.0f,
//    },
//    .speed_pid = {
//        .Kp = 0.0f,
//        .Ki = 0.0f,
//        .Kd = 0.0f,
//        .limit = 500.0f,
//        .output_limit = 200.0f,
//    }
//};


//R_down_PID R_DOWN_PID = {
//    .pos_pid = {
//        .Kp = 0.0f,
//        .Ki = 0.0f,
//        .Kd = 0.0f,
//        .limit = 1000.0f,
//        .output_limit = 500.0f,
//    },
//    .speed_pid = {
//        .Kp = 0.0f,
//        .Ki = 0.0f,
//        .Kd = 0.0f,
//        .limit = 500.0f,
//        .output_limit = 200.0f,
//    }
//};
float motor1_angle = 0.0f;
float motor2_angle = 0.0f;

Vec2 O1 = { 0.0875f, 0.21841f };
Vec2 O2 = { 0.2775f, 0.11341f };
Vec2 Cw = { 0.0f , 0.0f };
Vec2 Op = { 0.0f , 0.0f }; 

int flag = 0;
int take = 0;
static uint8_t traj_started = 0;

TaskHandle_t Volleyball_Serve_Handle; 
void Volleyball_Serve(void *pvParameters)
{
	  vTaskDelay(2000);
    RobStrideInit(&R_up, &hcan1, 0x01, RobStride_04);
	  RobStrideInit(&R_down, &hcan1, 0x02, RobStride_04);
	
//	  RobStrideSetMode(&R_up, RobStride_Torque);
//	  RobStrideSetMode(&R_down, RobStride_Torque);
	  RobStrideSetMode(&R_up, RobStride_MotionControl);
	  RobStrideSetMode(&R_down, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_up);
	  RobStrideEnable(&R_down);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_up);
    RobStrideResetAngle(&R_down);
	
		TickType_t last_wake = xTaskGetTickCount();
    for(;;)
    {  
			float theta = atan2f(-(Cw.x - Op.x),(Cw.y - Op.y));
			
			float c = cosf(theta);
			float s = sinf(theta);

			Vec2 P1w = {
					Op.x - HALF_P * c,
					Op.y - HALF_P * s
			};

			Vec2 P2w = {
					Op.x + HALF_P * c,
					Op.y + HALF_P * s
			};
			
			Vec2 r1 = {
			P1w.x - O1.x,
			P1w.y - O1.y
			};

			float d1 = sqrtf(r1.x*r1.x + r1.y*r1.y);
			float phi1 = atan2f(r1.y, r1.x);
			float alpha1 = acosf((L_OA*L_OA + d1*d1 - L_AP*L_AP) / (2.0f * L_OA * d1));

		  motor1_angle = phi1 + alpha1;
			
			Vec2 r2 = {
			P2w.x - O2.x,
			P2w.y - O2.y
			};

			float d2 = sqrtf(r2.x * r2.x + r2.y * r2.y);
			float phi2 = atan2f(r2.y, r2.x);
			float alpha2 = acosf((L_OA*L_OA + d2*d2 - L_AP*L_AP) / (2.0f * L_OA * d2));

			motor2_angle = phi2 - alpha2;  
			
		if (take == 1 && traj_started == 0)   // 第一次触发
		{
				Cubic_SetTrajectory(
						&traj_up,
						R_up.state.rad,        // 当前真实角度
						R_up.state.omega,      // 当前真实速度
						motor1_angle,          // 目标角度
						0,
						0.2f,                  // 300ms
						xTaskGetTickCount()
				);

				Cubic_SetTrajectory(
						&traj_down,
						R_down.state.rad,
						R_down.state.omega,
						motor2_angle,
						0,
						0.2f,
						xTaskGetTickCount()
				);
				traj_started = 1;
		}

		if(take == 1)
  		{
				Cubic_GetFullState(&traj_up,   xTaskGetTickCount(), &traj_up_state);
				Cubic_GetFullState(&traj_down, xTaskGetTickCount(), &traj_down_state);

				RobStrideMotionControl(&R_up, 0x01,
						R_up_expect.expect_torque,
						traj_up_state.pos,
						traj_up_state.vel,
						R_up_expect.kp,
						R_up_expect.kd);

				RobStrideMotionControl(&R_down, 0x02,
						R_down_expect.expect_torque,
						traj_down_state.pos,
						traj_down_state.vel,
						R_down_expect.kp,
						R_down_expect.kd);

			}
//        PID_Control2(R_up.state.rad, R_up_expect.expect_angle, &R_UP_PID.pos_pid);
//			  PID_Control2(R_down.state.rad, R_down_expect.expect_angle, &R_DOWN_PID.pos_pid);
//        vTaskDelay(1);
//        PID_Control2(R_up.state.omega, R_UP_PID.pos_pid.pid_out, &R_UP_PID.speed_pid);
//        PID_Control2(R_down.state.omega, R_DOWN_PID.pos_pid.pid_out, &R_DOWN_PID.speed_pid);

			if(flag == 0 && take == 0){				
//				RobStrideTorqueControl(&R_up, 0);
//				RobStrideTorqueControl(&R_down, 0);
				RobStrideMotionControl(&R_up, 0x01, 0, R_up.state.rad, 0, 0, 0);
				RobStrideMotionControl(&R_down, 0x02, 0, R_down.state.rad, 0, 0, 0);
				
				traj_started = 0;

//			}
//			if(flag == 1){
////				RobStrideTorqueControl(&R_up, R_UP_PID.speed_pid.pid_out);
////				RobStrideTorqueControl(&R_down, R_DOWN_PID.speed_pid.pid_out);
//RobStrideMotionControl(&R_up, 0x01, R_up_expect.expect_torque, R_up_expect.expect_angle, R_up_expect.expect_omega, R_up_expect.kp, R_up_expect.kd);
//RobStrideMotionControl(&R_down, 0x02, R_down_expect.expect_torque, R_down_expect.expect_angle, R_down_expect.expect_omega, R_down_expect.kp, R_down_expect.kd);
//			}

}
       vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
	RobStrideRecv_Handle(&R_up, &hcan1, ID, buf);
  RobStrideRecv_Handle(&R_down, &hcan1, ID, buf);
}
