#include "uart.h"
#include <string.h>   // strncmp strcpy.....

#define COMMAND_NUMBER 20
#define COMMAND_LENGTH 40

void pc_command_processing(void);
void bt_command_processing(void);

const char help_cmd[][20] =
{
"--- command list ---\n",
	 "\tled_all_on\n",
	 "\tled_all_off\n",
	 "\tled_left_on\n",
	 "\tled_right_on\n",
	 "\tled_flower_on\n",
	 "\tled_flower_off\n",
	 "\tled_left_keepon\n",
	 "\tled_right_keepon\n"
};

//------------ PC -------------------------
volatile uint8_t rx_buffer[COMMAND_NUMBER][COMMAND_LENGTH];
volatile int rear=0;
volatile int front=0;

extern UART_HandleTypeDef huart2;
extern uint8_t rx_data;  // uart rx byte
//----------------------------------------
//------------- BT ---------------------------
volatile uint8_t rx_btbuffer[COMMAND_NUMBER][COMMAND_LENGTH];
volatile int rear_bt=0;
volatile int front_bt=0;

extern UART_HandleTypeDef huart1;
extern uint8_t rx_btdata;  // uart rx byte
//-------------------------------------------
extern int func_index;
extern void (*funcp[])();
// move Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c to here
// comportamaster로부터 1 char를 수신할때 마다 이곳으로 자동 진입된다.
// led_flower_on\n
// 9600bps 인경우는 HAL_UART_RxCpltCallbackt 수행후 1ms안에는 빠져나가야 한다.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	volatile static int i=0;
	volatile static int i_bt=0;

	if (huart == &huart2)
	{
		uint8_t data;

		data = rx_data;

		if (data == '\n')
		{
			rx_buffer[rear][i++] = '\0';  // 문장의 끝을 NULL로
			i=0;
			rear++;
			rear %= COMMAND_NUMBER;
			// !!!! queue full 체크하는 logic이 들어 가야 한다. !!!!!
		}
		else
		{
			rx_buffer[rear][i++] = data;  //(1) rx_buffer[rear][i] = data;
										  //(2) i++
		}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);  // uart2		                                            // 반드시 집어 넣어야 다음 INT발생
	}
	else if (huart == &huart1)  	//------------- BT ------------------
	{
		uint8_t data;

		data = rx_btdata;

		HAL_UART_Receive_IT(&huart1, &rx_btdata, 1);  // uart1		                                            // 반드시 집어 넣어야 다음 INT발생
	}
	//---------------------------------------------
}

void pc_command_processing(void)
{
	if (front != rear)  // rx_buff에 데이터 존재
	{
		 // rx_buff[front] 와 동일 &rx_buff[front][0]
		if (strncmp(rx_buffer[front], "led_all_on", strlen("led_all_on")) == 0)
		{
			func_index=0;
		}
		else if (strncmp(rx_buffer[front], "led_all_off", strlen("led_all_off")) == 0)
		{
			func_index=1;
		}
		else if (strncmp(rx_buffer[front], "led_left_on", strlen("led_left_on")) == 0)
		{
			func_index=2;
		}
		else if (strncmp(rx_buffer[front], "led_right_on", strlen("led_right_on")) == 0)
		{
			func_index=3;
		}
		else if (strncmp(rx_buffer[front], "led_flower_on", strlen("led_flower_on")) == 0)
		{
			func_index=4;
		}
		else if (strncmp(rx_buffer[front], "led_flower_off", strlen("led_flower_off")) == 0)
		{
			func_index=5;
		}
		else if (strncmp(rx_buffer[front], "led_left_keepon", strlen("led_left_keepon")) == 0)
		{
			func_index=6;
		}
		else if (strncmp(rx_buffer[front], "led_right_keepon", strlen("led_right_keepon")) == 0)
		{
			func_index=7;
		}
		else if (strncmp(rx_buffer[front], "help", strlen("help")) == 0)
		{
			for (int i=0; i < 9; i++)
				printf("%s", help_cmd[i]);
		}
		front++;
		front %= COMMAND_NUMBER;
		// !!!!! queue full check 하는 기능 추가 요망 !!!!!
	}
}

void bt_command_processing(void)
{
	if (front_bt != rear_bt)  // rx_buff에 데이터 존재
	{
		 // rx_buff[front] 와 동일 &rx_buff[front][0]
		if (strncmp(rx_btbuffer[front_bt], "led_all_on", strlen("led_all_on")) == 0)
		{
			func_index=0;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_all_off", strlen("led_all_off")) == 0)
		{
			func_index=1;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_left_on", strlen("led_left_on")) == 0)
		{
			func_index=2;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_right_on", strlen("led_right_on")) == 0)
		{
			func_index=3;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_flower_on", strlen("led_flower_on")) == 0)
		{
			func_index=4;
		}
		else if (strncmp(rx_buffer[front_bt], "led_flower_off", strlen("led_flower_off")) == 0)
		{
			func_index=5;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_left_keepon", strlen("led_left_keepon")) == 0)
		{
			func_index=6;
		}
		else if (strncmp(rx_btbuffer[front_bt], "led_right_keepon", strlen("led_right_keepon")) == 0)
		{
			func_index=7;
		}
		front_bt++;
		front_bt %= COMMAND_NUMBER;
		// !!!!! queue full check 하는 기능 추가 요망 !!!!!
	}
}







