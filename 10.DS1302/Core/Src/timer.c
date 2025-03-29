#include "timer.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

void delay_us(int us);	//	입력받은 us만큼 Delay 시키는 함수
extern void delay_ms(int us);

volatile int TIM11_1ms_counter = 0;
volatile int TIM11_DHT11_counter = 0;
volatile int TIM11_DS1302 = 0;

//	Move from Dirver/STM32F4XX_HAL_Driver/src/stm32f4xx_hal_tim.c

//	Timer Interrupt가 발생할 때 마다, 해당 함수로 이동한다.
//	Interrupt Call Back Function 혹은 Interrupt Service Routine이라 한다.
// 	Interrupt Call Back Function은 가능한 짧게 구현한다.
// 	Interrupt 내부의 변수는 최적화 방지를 위해 Volatile로 선언한다.



// 	Ex. tim11의 경우 1ms마다 해당 함수로 진입한다.


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM11)		//	TIM11 is 1MHz.(1us)
	{
		TIM11_1ms_counter++;
		TIM11_DS1302++;
		TIM11_DHT11_counter++;
	}


}


void delay_us(int us)
{

	__HAL_TIM_SET_COUNTER(&htim2, 0);	//	TIM2의 COUNTER를 RESET.

	while(__HAL_TIM_GET_COUNTER(&htim2) < us)	//	사용자가 지정한 us만큼 Delay.
		;


}
