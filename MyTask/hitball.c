#include "hitball.h"
#include "Task_Init.h"


GPIO_PinState key1, key2, key3, key4;

uint8_t hit_ball_trigger = 0;
uint8_t flag_one = 0;

TaskHandle_t Volleyball_Serve_Handle; 
void Volleyball_Serve(void *pvParameters)
{		
	
	static uint8_t prev_keys_none = 1;
	
	TickType_t last_wake_time = xTaskGetTickCount();	  
  for(;;)
	  {		
			
//		//∂¡»°µÁ∆Ω 
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

		uint8_t current_any = (key1 == GPIO_PIN_SET) || 
													(key2 == GPIO_PIN_SET) || 
													(key3 == GPIO_PIN_SET) || 
													(key4 == GPIO_PIN_SET);
		if (prev_keys_none && current_any)
		{
			flag_one = 1;
		}
		prev_keys_none = ! current_any;

			
		if(flag_one == 1)
		{
		hit_ball_trigger = 1;
		}
		
		if(hit_ball_trigger == 1)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
				vTaskDelay(300);
				
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);

				
				hit_ball_trigger = 0;
				flag_one = 0;
			}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		 }
}
