#include "main.h"
#include "button.h"
#include "buzzer.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;
extern volatile int TIM11_1ms_servo_motor;
extern int get_button(GPIO_TypeDef* GPIO, uint16_t GPIO_Pin, int button_number);
extern volatile int TIM4_10ms_counter;
extern void beep(int repeat, int Hz, int time);

void servo_motor_main(void);
void servo_motor_button(void);

// 1초에 한번씩 180 --> 90 --> 0 회전
/*
 system clock : 84MHZ (84000000HZ)
 TIM3 50KHZ 입력 : 84000000 / 1680  --> 50KHZ
 분주비 입력이 16bit 이상이기 때문
 T = 1/f = 1/50000 -> 20us
 20ms = 20us x 1000
 ---------<Servo Motor SG90의 Spec>------------
 2ms(180) : 20us x 100
 1.5ms(90) : 20us x 75
 1ms(0) : 20us x 50
 */

void servo_motor_main(void)
{
	 HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);   // buzzer
	beep(1,783,600);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 75);
	HAL_Delay(1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 125);
}

