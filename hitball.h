#ifndef __HITBALL_H_
#define __HITBALL_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "PID_old.h"

#define MAX_TORQUE 120.0f
#define normal_torque 40.0f
#define MAX_SIZE 40.0f

#define C_OFFSET_Y   0.0645f     // C在平台坐标中的y
#define HALF_P       0.1f    // P1-P2 一半
#define L_OA           0.27f    // O1-A = O2-B
#define L_AP           0.24f    // A-P1 = B-P2

typedef struct {
    float x;
    float y;
} Vec2;


extern TaskHandle_t Volleyball_Serve_Handle;
typedef struct
{
  float expect_torque;
	float expect_angle;
	float expect_omega;
//motioncontrol再用
	float kp;
	float kd;
}RobStride_Expect;


//较高处电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_up_PID;

//较低处电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_down_PID;

void Volleyball_Serve(void *argument);

#endif

