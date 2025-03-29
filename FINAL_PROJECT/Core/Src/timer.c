#include "main.h"

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
extern uint8_t keypadScan();
extern void insert_queue(uint8_t value);
//delay_us(10); // 10us동안 wait
void delay_us(int us);

// interrupt call back function(interrupt service routine 이라 한다.)
// interrupt 내부에 쓰는 변수는 최적화 방지를 위해 변수 type앞에
// volatile이라고 선언한다
volatile int TIM11_1ms_counter=0;
volatile int TIM11_1ms_ds1302=0;
volatile int TIM11_1ms_servo_motor=0;
volatile int TIM4_10ms_counter=0;
volatile int TIM11_1ms_dot=0;
// Driver/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_tim.c 에서 move
// timer interrupt가 뜰때 마다 이곳으로 자동적으로 들어 온다.
// 예) tim11 인 경우 1ms마다 이곳으로 진입한다.
// - prescale :84 -counter period : 1000
// T = 1/f 1/100000Hz = 0.000001 sec
// interrupt call back function은 가능한 짧게 구현한다


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM11){ //ADD_JONGHWAN
		TIM11_1ms_counter++;
		TIM11_1ms_ds1302++;
		TIM11_1ms_servo_motor++;
		TIM11_1ms_dot++;
	}
	else if(htim->Instance==TIM4){ //ADD_JONGHWAN
			TIM4_10ms_counter++; //10ms
			if(TIM4_10ms_counter>=6)
			{
				unsigned char key_value;

				TIM4_10ms_counter=0;
				key_value = keypadScan();
				if(key_value)
				{
					insert_queue(key_value); //circular queue에 넣는다
				}
			}

		}
}


// 1MHZ 분주 주파수가 TIM2으로 입력된다.
// 한 주기가 T=1/f   1/1000000HZ ==> 0.000001sec (1us)
// 예) delay_us(1000) --> 1ms동안 wait
void delay_us(int us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0); //timer2번의 카운터 클리어

	// 사용자가 지정한 us동안 머물러 있도록 한다.
	while(__HAL_TIM_GET_COUNTER(&htim2)<us)
		;   //no operation
}
