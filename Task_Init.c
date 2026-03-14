#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "hitball.h"

void Updatekey(Remote_Handle_t * xx) 
	{ 
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
	}
	
void Task_Init(){
	 //Ò£¿ØÆ÷
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));

	
	vPortEnterCritical();
	
	xTaskCreate(Remote,
         "Remote",
          400,
          NULL,
          3,
          &Remote_Handle); 
	
	xTaskCreate(Move_Remote,
         "Move_Remote",
          256,
          NULL,
          3,
          &Move_Remote_Handle);
					
//	xTaskCreate(Volleyball_Serve,
//         "hit_ball",
//          256,
//          NULL,
//          3,
//          &Volleyball_Serve_Handle);
					
	vPortExitCritical();
}
