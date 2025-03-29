#include "main.h"


void ultrasonic_processing(void);
void make_trigger(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

extern volatile int TIM11_Ultrasonic;
extern TIM_HandleTypeDef htim1;

volatile uint8_t is_first_capture = 0;	// 0 : RISING EDGE, 1: FALLING EDGE
volatile int distance = 0;	//	거리 측정한 PULSE 갯수를 저장하는 변수.
volatile int ic_cpt_finish_flag = 0;	//	초음파 거리 측정 완료 되었는지 확인하기 위한 FLAG




void ultrasonic_processing(void)
{
	int dis = 0;
	char lcd_buff[20];

	if(TIM11_Ultrasonic >= 1000)
	{
		TIM11_Ultrasonic = 0;
		make_trigger();


		if(ic_cpt_finish_flag)
		{
			ic_cpt_finish_flag = 0;
			dis = distance;
			dis = dis * 0.034 / 2;	//	초음파는 1us당 0.034sec를 이동한다. ppt 참조.
			printf("DISTANCE : %d cm\n\n", dis);

			sprintf(lcd_buff, "DISTANCE : %dcm   ", dis);
			move_cursor(1,0);
			lcd_string(lcd_buff);
		}

	}
}




void make_trigger(void)
{
	HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET);
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)	//	ECHO의 RISING EDGE, FALLING EDGE 시, 이 함수로 진입한다.
{
	if(htim->Instance == TIM1)
	{

		if(is_first_capture == 0)	// RISING EDGE?
		{
			__HAL_TIM_SET_COUNTER(htim, 0);		//	COUNTER 초기화
			is_first_capture = 1;	//	RISING EDGE를 만났다는 FLAG 변수를 SET.
		}
		else if(is_first_capture == 1)	// FALLING EDGE?
		{
			is_first_capture = 0;	//	다움 ECHO PULSE를 COUNT 하기 위해 CLEAR.
			distance = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			ic_cpt_finish_flag = 1; //	초음파 거리 측정 완료
		}

	}
}
