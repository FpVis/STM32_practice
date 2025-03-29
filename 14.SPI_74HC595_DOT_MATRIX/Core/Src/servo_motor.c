#include "main.h"
#include "button.h"

extern TIM_HandleTypeDef htim3;
extern volatile int TIM11_SERVO_MOTOR_counter;


void servo_motor_main(void);



/*
 * 1초에 1번씩 180 -> 90 -> 0 회전
 *
 * SYSTEM CLOCK : 84 MHz
 * TIM3 50 KHz 입력 : 84 MHz / 1680 --> 50 KHz
 * T = 1 / f = 1 / 50000 ==> 0.00002 sec (20us)
 *
 * 180도(2ms) : 20us * 100
 * 90도(2ms) : 20us * 75
 * 0도(2ms) : 20us * 50
 *
 */
void servo_motor_main(void)
{
	static int servo_state = 0;

		if(get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 83);	// 90
			HAL_Delay(3000);
			servo_state = 0;
		}


		if(TIM11_SERVO_MOTOR_counter >= 1000 )
		{

			TIM11_SERVO_MOTOR_counter = 0;

			switch(servo_state)
			{
			case 0:
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 133);	// 0
				break;
			case 1:
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 83);	// 90
				break;
			case 2:
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 30);	// 180
				break;
			}

			servo_state = (servo_state + 1) % 3;


		}

}
