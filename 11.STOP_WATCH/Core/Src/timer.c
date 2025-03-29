#include "timer.h"

extern TIM_HandleTypeDef htim11;

volatile int TIM11_1ms_counter = 0;
volatile int TIM11_DS1302 = 0;
volatile int TIM11_STOPWATCH = 0;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM11)		//	TIM11 is 1MHz.(1us)
	{
		TIM11_1ms_counter++;
		TIM11_DS1302++;
		TIM11_STOPWATCH++;
	}


}

