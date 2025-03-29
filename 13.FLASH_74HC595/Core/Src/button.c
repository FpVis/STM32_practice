#include "button.h"

extern volatile int TIM11_1ms_counter;


unsigned char button_status[BUTTON_NUMBER] = {
		BUTTON_RELEASE,	BUTTON_RELEASE, BUTTON_RELEASE, BUTTON_RELEASE
};

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
