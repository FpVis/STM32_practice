#include "keypad.h"
#include "circularQueue.h"

#define GPIOB_START_ADDR 0x40020400
#define GPIOC_START_ADDR 0x40020800

GPIO_TypeDef* keypadRowPort[4] = {GPIOC, GPIOC, GPIOC, GPIOC}; //R1~R4
GPIO_TypeDef* keypadColPort[4] = {GPIOB, GPIOB, GPIOB, GPIOB}; //C1~C4
uint16_t keypadRowPin[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3}; //R1~R4 GPIO Input & Pull-up으로 설정을 해야 한다.
uint16_t keypadColPin[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}; //C1~C4  GPIO Output으로만 설정 한다.

extern uint8_t input_mode;
int input_num=0;
char key_value;
int start = 0;
extern unsigned char queue[QUEUE_MAX];
extern void show_menu(void);
extern void queue_init();
extern int menu_flg;

const uint8_t keypadColPinNum[4] = {12, 13, 14, 15}; // 각 열 핀 번호
const uint8_t keypadRowPinNum[4] = {0, 1, 2, 3}; // 각 열 핀 번호

void keypadInit()
{
    for (uint8_t col = 0; col < 4; col++)		// 	HAL_GPIO_WritePin(keypadColPort[col], keypadColPin[col], SET);
    {
        *(volatile unsigned int *)(GPIOB_START_ADDR + 0x14) |= (1 << keypadColPinNum[col]);
    }
}


uint8_t getKeypadState(uint8_t col, uint8_t row)
{
#if 1
	uint8_t keypadChar[4][4] = {
			{'/', '3', '2', '1'},
			{'*', '6', '5', '4'},
			{'-', '9', '8', '7'},
			{'+', '=', '0', ' '},
	};

#else
	uint8_t keypadChar[4][4] = {
			{'1', '2', '3', '/'},
			{'4', '5', '6', '*'},
			{'7', '8', '9', '-'},
			{' ', '0', '=', '+'},
	};
#endif
	static uint8_t prevState[4][4] = {
			{1, 1, 1, 1},
			{1, 1, 1, 1},
			{1, 1, 1, 1},
			{1, 1, 1, 1},
	};
	uint8_t curState = 1;

    *(volatile unsigned int *)(GPIOB_START_ADDR + 0x14) &= ~(1 << keypadColPinNum[col]);

	// curState = HAL_GPIO_ReadPin(keypadRowPort[row], keypadRowPin[row]);
	curState = (*(volatile unsigned int *)(GPIOC_START_ADDR+ 0x10) >> keypadRowPinNum[row]) & 0x01;


	*(volatile unsigned int *)(GPIOB_START_ADDR + 0x14) |= (1 << keypadColPinNum[col]);

	if(curState == PUSHED && prevState[col][row] == RELEASED)
	{
		prevState[col][row] = curState;
		return 0;
	}
	else if (curState == RELEASED && prevState[col][row] == PUSHED)
	{
		prevState[col][row] = curState;
		return keypadChar[col][row];
	}
	return 0;
}

uint8_t keypadScan()
{
	uint8_t data;

	for(uint8_t col=0; col<4; col++)
	{
		for(uint8_t row=0; row<4; row++)
		{
			data = getKeypadState(col, row);
			if(data != 0)
			{
				return data;
			}
		}
	}
	return 0;
}

void keypad_main()
{
	static char keypad_buff[100]={};
	if(queue_empty() != 1)
	{
		input_mode=1;

		if(!start) // 키패드 첫 입력때
		{
			lcd_init();
			lcd_set_cur(0, 0);
			start=1;

			show_menu();
		}
		if(menu_flg)
	    select_menu();

		key_value = read_queue();
		sprintf(keypad_buff,"%c",key_value);
		lcd_send_string(keypad_buff);

	}

}
//
//void find_num(int input_num)
//{
//	t_st *p = list<-head;
//	while(p != NULL)
//	{
//		if(p->num == input_num)
//		{
//			if(p->enter == NULL)
//			{
//				printf("enter\n");
//				p->enter.hour=ds_time.hours;
//				p->enter.min=ds_time.minutes;
//				p->enter.sec=ds_time.seconds;
//			}
//			else
//			{
//				printf("exit\n");
//				p->exit.hour=ds_time.hours;
//				p->exit.min=ds_time.minutes;
//				p->exit.sec=ds_time.seconds;
//			}
//		}
//	}
//}
