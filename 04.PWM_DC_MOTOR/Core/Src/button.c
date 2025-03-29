
#include "button.h"


extern void led_main(void);
extern void led_left_on(void);
extern void led_right_on(void);
extern void led_left_keep_on(void);
extern void led_right_keep_on(void);
extern void led_flower_on(void);
extern void led_flower_off(void);


extern void on_led(void);
extern void off_led(void);
extern void(*funcp[])();


int func_index = -1;





extern volatile int TIM11_1ms_counter;

// none OS 방식 또는 Loop Monitor 방식이라 한다.


unsigned char button_status[BUTTON_NUMBER] = {
		BUTTON_RELEASE,	BUTTON_RELEASE, BUTTON_RELEASE, BUTTON_RELEASE
};


void button_check(void)
{
	static uint8_t demo_led = 0;
	static uint8_t btn1 = 0;
	static uint8_t btn2 = 0;
	static uint8_t btn3 = 0;
	static uint8_t working_btn = -1;

	if(get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)
	{
		HAL_GPIO_WritePin(GPIOB, 0xff, 1);
		working_btn = BUTTON0;
		func_index = -1;
	}

	if(get_button(GPIOC, GPIO_PIN_6, BUTTON1) == BUTTON_PRESS)
	{
		btn1 = !btn1;
		working_btn = BUTTON1;
		func_index = -1;
	}

	if(get_button(GPIOC, GPIO_PIN_8, BUTTON2) == BUTTON_PRESS)
	{
		btn2 = !btn2;
		working_btn = BUTTON2;
		func_index = -1;
	}

	if(get_button(GPIOC, GPIO_PIN_9, BUTTON3) == BUTTON_PRESS)
	{
		btn3 = !btn3;
		working_btn = BUTTON3;
		func_index = -1;
	}


	if(func_index == -1)
	{
		if(working_btn == BUTTON1)

		{
			if(btn1 == 0)

				led_flower_on();

			else

				led_flower_off();

		}


		if (working_btn == BUTTON2)
		{
			if (btn2 == 0)

				led_left_on();

			else

				led_right_on();

		}



		if (working_btn == BUTTON3)
		{
			if (btn3 == 0)

					led_left_keep_on();

			else

					led_right_keep_on();

		}
	}

}

int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin, int button_number)		//	 TO PREVENT SWITCH BOUNCE
{
	int state = HAL_GPIO_ReadPin(GPIO, GPIO_Pin);	//	ACTIVE : 0		INACTIVE : 1


	if(state == BUTTON_PRESS && button_status[button_number] == BUTTON_RELEASE)
	{
		if(TIM11_1ms_counter >= 60)
					{
					TIM11_1ms_counter = 0;
					}

		button_status[button_number] = BUTTON_PRESS;
		return BUTTON_RELEASE;		//		1
	}

	else if(state == BUTTON_RELEASE && button_status[button_number] == BUTTON_PRESS)
	{
		button_status[button_number] = BUTTON_RELEASE;

		if(TIM11_1ms_counter >= 60)
							{
							TIM11_1ms_counter = 0;
							}
		return BUTTON_PRESS;		//		0
	}

	return BUTTON_RELEASE;	//	버튼이 눌려진 상태가 아님

}
