#include "timer.h"
#include "main.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim4;

extern keypadScan();


volatile int TIM11_1ms_counter = 0;
volatile int TIM4_10ms_counter = 0;
volatile int TIM11_SERVO_MOTOR_counter = 0;
volatile int TIM11_DS1302 = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM11)		//	TIM11 is 1MHz.(1us) --> COUNTER PERIOD 1000 --> 1kHz --> 1ms
	{
		TIM11_1ms_counter++;
		TIM11_SERVO_MOTOR_counter++;
		TIM11_DS1302++;
	}


	if(htim->Instance == TIM4)		//	TIM4 is 1MHz.(1us) --> COUNTER PERIOD 100 --> 10kHz --> 0.1ms
	{
		TIM4_10ms_counter++;


		if(TIM4_10ms_counter >= 6)		//	60ms마다 스캔 진행 (디바운스 방지)
		{
			unsigned char key_value;

			TIM4_10ms_counter = 0;
			key_value = keypadScan();

			if(key_value)
			{
				insert_queue(key_value);	//	CIRCULAR QUEUE에 집어 넣는다.
			}

		}

	}

}



void DEMO_LED(void)
{
	if(TIM4_10ms_counter >= 500)
	{
		TIM4_10ms_counter = 0;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}
