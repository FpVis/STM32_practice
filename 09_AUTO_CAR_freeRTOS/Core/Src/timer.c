#include "main.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
// delay_us(10);  // 10us동안 wait
void delay_us(int us);



// 1MHZ 분주 주파수가 TIM2으로 입력 된다.
// T=1/f 1/1000000HZ ==> 0.000001sec (1us) : 1개의 펄스 소요 시간
// 예)  delay_us(1000) --> 1ms동안 wait
void delay_us(int us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // timer2번의 counter를 clear

	// 사용자가 지정한 us동안 머물러 있도록 한다.
	while(__HAL_TIM_GET_COUNTER(&htim2) < us)
		;   // NO Operation
}




