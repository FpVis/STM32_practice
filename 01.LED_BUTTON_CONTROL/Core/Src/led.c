#include "led.h"

void led_main(void);
void led_left_on(void);
void led_right_on(void);
void led_left_keep_on(void);
void led_right_keep_on(void);
void led_flower_on(void);
void led_flower_off(void);
void on_led(int led_num);
void off_led(int led_num);

extern void button_check(void);

void led_main(void)

{
	while(1)
		{
		button_check();
		}
}

void led_left_on(void)	//	7 6 5 4 3 2 1 0 @ LED 순차적으로, 200ms 주기로 동작 시키는 함수
						//					@ 기존 On LED는 OFF 처리
{
	static int i = 0;

	if(i < 8)
			{
				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
				HAL_Delay(100);
			}
			i++;

	if(i >= 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);
			}
}


void led_right_on(void)
{
	static int i = 0;

			if(i < 8)
			{
				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_GPIO_WritePin(GPIOB, 0x80 >> i, 1);
				HAL_Delay(100);
			}
			i++;

			if(i >= 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);
			}
}


void led_left_keep_on(void)
{
	static int i = 0;

			if(i < 8)
			{
				HAL_GPIO_WritePin(GPIOB, 0x01 << i, 1);
				HAL_Delay(100);
			}
			i++;

			if(i >= 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);
			}

}


void led_right_keep_on(void)
{
	static int i = 0;

			if(i < 8)
			{
				HAL_GPIO_WritePin(GPIOB, 0x80 >> i, 1);
				HAL_Delay(100);
			}
			i++;

			if(i >= 8)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);
			}
}


void led_flower_on(void)
{
	static int i = 0;

			if(i < 4)
			{
				HAL_GPIO_WritePin(GPIOB, 0X10 << i | 0x08 >> i, 1);
				HAL_Delay(100);

			}
			i++;

			if(i >= 4)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);

			}
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




			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOB, 0xff, 1);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, 0x01 << i | 0x80 >> i, 0);
				HAL_Delay(100);

			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, 0x01 << i | 0x80 >> i, 0);
				HAL_Delay(100);
			}
			i++;

			if(i >= 4)
			{
				i = 0;

				HAL_GPIO_WritePin(GPIOB, 0xff, 0);
				HAL_Delay(100);

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

void on_led(int led_num)
{
	HAL_GPIO_WritePin(GPIOB, 0x01 << led_num, 1);
}


void off_led(int led_num)
{
	HAL_GPIO_WritePin(GPIOB, 0x01 << led_num, 0);

}








