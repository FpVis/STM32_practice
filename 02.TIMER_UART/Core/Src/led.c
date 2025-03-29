#include "led.h"

void led_main(void);
void on_led(void);
void off_led(void);
void led_left_on(void);
void led_right_on(void);
void led_left_keep_on(void);
void led_right_keep_on(void);
void led_flower_on(void);
void led_flower_off(void);



extern void pc_command_processing(void);

extern void button_check(void);
extern void delay_us(int);
extern void dht11_main(void);


extern volatile int TIM11_DHT11_counter;
extern volatile int TIM11_1ms_counter;

extern func_index;






void(*funcp[])() =
{
		on_led,  // 0
		off_led,
		led_left_on,
		led_right_on,
		led_left_keep_on,
		led_right_keep_on,
		led_flower_on,
		led_flower_off,
};



void led_main(void)

{
	while(1)
	{



			button_check();
			pc_command_processing();
			dht11_main();



//-----------------DEMO LED------------------//
//		if(TIM11_1ms_counter >= 50)		//		TIM11_1ms_counter == 50으로 선언하지 않는 것은, TIM11_1ms_counter 값이 50 이상으로 튀는 경우를 방지하기 위해서이다.
//		{
//				TIM11_1ms_counter = 0;
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		}
//-----------------delay_us() VERIFYING------------------//


//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	delay_us(500000);


	}



}





void led_left_on(void)	//	7 6 5 4 3 2 1 0 @ LED 순차적으로, 200ms 주기로 동작 시키는 함수
						//					@ 기존 On LED는 OFF 처리
{
#if 1
	static int i = 0;

	if(TIM11_1ms_counter >= 100)
			{
			TIM11_1ms_counter = 0;
			HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				if(i < 8)

					{

						HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);



					}
					i++;

					if(i >= 8)
					{
						i = 0;

						HAL_GPIO_WritePin(GPIOB, 0xff, 0);

					}
			}


#else
	for(int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOB, 0xff, 0);
	}
#endif
}




void led_right_on(void)
{
#if 1
	static int i = 0;
	if(TIM11_1ms_counter >= 100)
		{
			TIM11_1ms_counter = 0;
			HAL_GPIO_WritePin(GPIOB, 0xff, 0);

			if(i < 8)
			{

				HAL_GPIO_WritePin(GPIOB, 0x80 >> i, 1);



			}
			i++;

			if(i >= 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);


			}

		}
#else
	for(int i = 0; i < 8 ; i--)
	{
		HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOB, 0xff, 0);
	}
#endif
}





void led_left_keep_on(void)
{
#if 1
	static int i = 0;



	if(TIM11_1ms_counter >= 100)
		{
		TIM11_1ms_counter = 0;


			if(i < 8)
			{

				HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);



			}
			i++;

			if(i > 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);


			}
		}

#else
	for(int i = 0; i < 8 ; i++)
	{
		HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
		HAL_Delay(200);
	}
	HAL_GPIO_WritePin(GPIOB, 0xff, 0);
	HAL_Delay(200);
#endif
}





void led_right_keep_on(void)
{
#if 1
	static int i = 0;


	if(TIM11_1ms_counter >= 100)
		{
		TIM11_1ms_counter = 0;


			if(i < 8)
			{

				HAL_GPIO_WritePin(GPIOB, 0x80 >> i, 1);


			}
			i++;

			if(i > 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);

			}
		}

#else
	for(int i = 8 ; i >0 ; i--)
		{
			HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(GPIOB, 0xff, 0);
		HAL_Delay(200);
#endif
}





void led_flower_on(void)
{
#if 1
	static int i = 0;
	if(TIM11_1ms_counter >= 100)
		{
		TIM11_1ms_counter = 0;
			if(i < 4)
			{
				HAL_GPIO_WritePin(GPIOB, 0X10 << i | 0x08 >> i, 1);


			}
			i++;

			if(i > 4)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);


			}
		}

#else
			for(int i = 0 ; i < 4 ; i++)
							{
								HAL_GPIO_WritePin(GPIOB, 0x01 << 3-i, 1);
								HAL_GPIO_WritePin(GPIOB, 0x01 << 4+i, 1);
								HAL_Delay(200);
							}
							HAL_GPIO_WritePin(GPIOB, 0xff, 0);
							HAL_Delay(200);



#endif
}
/*

void led_flower_on(void)
{
	for(int i = 0 ; i < 4 ; i++)
			{
				HAL_GPIO_WritePin(GPIOB, 0x10 << i | 0x08 >> i, 1);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(GPIOB, 0xff, 0);
			HAL_Delay(200);
}

*/

void led_flower_off(void)
{
#if 1
	static int i = 0;


	if(TIM11_1ms_counter >= 100)
		{
		TIM11_1ms_counter = 0;

			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOB, 0xff, 1);

				HAL_GPIO_WritePin(GPIOB, 0x01 << i | 0x80 >> i, 0);


			}

			else
			{
				HAL_GPIO_WritePin(GPIOB, 0x01 << i | 0x80 >> i, 0);

			}
			i++;

			if(i > 4)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);


			}

		}
#else
	HAL_GPIO_WritePin(GPIOB, 0xff, 1);
	HAL_Delay(200);
	for(int i = 0 ; i < 4 ; i++)
				{
					HAL_GPIO_WritePin(GPIOB, 0x01 << 7-i, 0);
					HAL_GPIO_WritePin(GPIOB, 0x01 << i, 0);
					HAL_Delay(200);
				}

				HAL_Delay(200);
#endif

}

/*

void led_flower_off(void)
{
	for(int i = 0 ; i < 4 ; i++)
			{
				HAL_GPIO_WritePin(GPIOB, 0x00 << i | 0x80 >> i, 1);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(GPIOB, 0xff, 0);
			HAL_Delay(200);
}

*/


void on_led(void)

{
	HAL_GPIO_WritePin(GPIOB, 0xff, 1);
}


void off_led(void)

{
	HAL_GPIO_WritePin(GPIOB, 0xff, 0);

}








