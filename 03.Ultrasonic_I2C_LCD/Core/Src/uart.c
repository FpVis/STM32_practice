#include "uart.h"
#include "string.h"		//		strncmp로 문자를 파싱하기 위해 선언한다.

#define COMMAND_NUMBER 20
#define COMMAND_LENGTH 40

void pc_command_processing(void);




volatile uint8_t rx_buffer[20][40];
volatile int rear = 0;
volatile int front = 0;

extern func_index;
extern UART_HandleTypeDef huart2;
extern uint8_t rx_data;		//		Interrupt가 발생되면 rx_data로 하나가 저장됨.
extern void (*funcp[])();


//	Copy from Dirver/STM32F4XX_HAL_Driver/src/stm32f4xx_hal_tim.c
//	Timer Interrupt가 발생할 때 마다, 해당 함수로 이동한다.
//	Interrupt Call Back Function 혹은 Interrupt Service Routine이라 한다.
// 	Interrupt Call Back Function은 가능한 짧게 구현한다.
// 	Interrupt 내부의 변수는 최적화 방지를 위해 Volatile로 선언한다.

//	9600 bps인 경우 하기 콜백 함수 수행 후 1ms 안에 빠져나가야함.
//	해당 함수는 Cirular Queue를 이용하여 COMMAND를 입력받는 기능을 구현함.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//		UART Call Back 함수 구현.
{
	volatile static int i = 0;


	if(huart == &huart2)
	{
		uint8_t data;
		data = rx_data;

		if(data == '\n')
			{
			rx_buffer[rear][i++] = '\0';	//	strncpy 사용 시, 문자열의 끝은 NULL이여야 복사가 가능하기 때문에 NULL 문자를 끝에 추가해준다.
			i = 0;
			rear++;
			rear %= COMMAND_NUMBER;

			}

		else
			{
			rx_buffer[rear][i++] = data;

			}

	}

	HAL_UART_Receive_IT(&huart2, &rx_data, 1);	//	UART2의 다음 Interrupt를 받기 위해 해당 함수를 실행한다.

}

void pc_command_processing(void)
{
	if (front != rear)		//		rx_buffer에 DATA가 존재하는 경우
	{
		printf("rx_buffer : %s\n", rx_buffer[front]);	// 인터럽트 펜딩이 발생하지 않도록, printf 구문은 인터럽트 루틴에 삽입하지 않는다.


		if (strncmp(rx_buffer[front], "on_led", strlen("on_led")) == 0)
		{
			func_index=0;

		}
		else if((strncmp(rx_buffer[front], "off_led", strlen("off_led")) == 0))
		{
			func_index=1;

		}
		else if((strncmp(rx_buffer[front], "led_left_on", strlen("led_left_on")) == 0))
		{
			func_index=2;

		}
		else if((strncmp(rx_buffer[front], "led_right_on", strlen("led_rignt_on")) == 0))
		{
			func_index=3;

		}
		else if((strncmp(rx_buffer[front], "led_left_keep_on", strlen("led_left_keep_on")) == 0))
		{
			func_index=4;

		}
		else if((strncmp(rx_buffer[front], "led_right_keep_on", strlen("led_right_keep_on")) == 0))
		{
			func_index=5;

		}
		else if((strncmp(rx_buffer[front], "led_flower_on", strlen("led_flower_on")) == 0))
		{
			func_index=6;

		}
		else if((strncmp(rx_buffer[front], "led_flower_off", strlen("led_flower_off")) == 0))
		{
			func_index=7;
		}
		front++;
		front %= COMMAND_NUMBER;
	}

   if (	func_index != -1)
   {
	   funcp[func_index] ();


   }


}

