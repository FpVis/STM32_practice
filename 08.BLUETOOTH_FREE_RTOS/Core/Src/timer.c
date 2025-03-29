#include "timer.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

void delay_us(int us);	//	입력받은 us만큼 Delay 시키는 함수
extern void delay_ms(int us);





void delay_us(int us)
{

	__HAL_TIM_SET_COUNTER(&htim2, 0);	//	TIM2의 COUNTER를 RESET.

	while(__HAL_TIM_GET_COUNTER(&htim2) < us)	//	사용자가 지정한 us만큼 Delay.
		;


}
