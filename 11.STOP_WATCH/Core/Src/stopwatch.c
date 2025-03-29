#include "main.h"
#include "button.h"
#include <time.h>

extern void ds1302_main();
extern void ds1302_gpio_init();
extern void	ds1302_init();


extern volatile int TIM11_1ms_counter;
extern volatile int TIM11_DS1302;
extern volatile int TIM11_STOPWATCH;


extern t_ds1302 ds_time;

uint8_t stopwatch_start_flag = 0;
uint8_t stopwatch_time_stop_flag = 0;
uint16_t stopwatch_time = 0;

char lcd_buff[20] = {0,};
char time_buff[20] = {0,};



void watch_main(void)
{
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);


	ds_time.hours = tm.tm_hour;
	ds_time.minutes = tm.tm_min;
	ds_time.seconds = tm.tm_sec;

	ds1302_gpio_init();
	ds1302_init();

	while(1)
	{


		//=================== WATCH MODE ===================//
		while(!stopwatch_start_flag)
		{
			ds1302_main();

			if(get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)
								{stopwatch_start_flag = !stopwatch_start_flag;}	//	스탑 워치 모드로 전환
		}


		//=================== STOPWATCH MODE ===================//

		while(stopwatch_start_flag)
		{

			if(TIM11_STOPWATCH >= 1000)
			{
				TIM11_STOPWATCH = 0;

				move_cursor(0,0);
				lcd_string("STOP WATCH          ");
				move_cursor(1,0);
				sprintf(time_buff, "%dM %dS           ", stopwatch_time/60, stopwatch_time%60);
				lcd_string(time_buff);												//	LCD 표현

				if(stopwatch_time_stop_flag)
				{stopwatch_time = stopwatch_time + 1;}

			}



			if(get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)		//	일반 시계 모드로 전환
			{
				stopwatch_start_flag = !stopwatch_start_flag;
				stopwatch_time = 0;
			}


			if(get_button(GPIOC, GPIO_PIN_6, BUTTON1) == BUTTON_PRESS)		//	일시정지 - 시작
			{
				stopwatch_time_stop_flag = !stopwatch_time_stop_flag;
			}


			if(get_button(GPIOC, GPIO_PIN_8, BUTTON2) == BUTTON_PRESS)		//	리셋
			{
				stopwatch_time = 0;
				stopwatch_time_stop_flag = 0;
			}


		}



	}
}
