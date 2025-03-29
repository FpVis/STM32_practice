#include "keypad.h"


GPIO_TypeDef* keypadRowPort[4] = {GPIOC, GPIOC, GPIOC, GPIOC}; //R1~R4
GPIO_TypeDef* keypadColPort[4] = {GPIOB, GPIOB, GPIOB, GPIOB}; //C1~C4
uint16_t keypadRowPin[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3}; //R1~R4 GPIO Input & Pull-up으로 설정을 해야 한다.
uint16_t keypadColPin[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}; //C1~C4  GPIO Output으로만 설정 한다.

uint8_t calculator();
unsigned char key_value[100];
unsigned char key_result;



void keypadInit()
{
	for(uint8_t col = 0; col < 4; col++)
	{
		HAL_GPIO_WritePin(keypadColPort[col], keypadColPin[col], SET); //초기 값 1로 셋팅
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

	HAL_GPIO_WritePin(keypadColPort[col], keypadColPin[col], RESET);
	curState = HAL_GPIO_ReadPin(keypadRowPort[row], keypadRowPin[row]);

	HAL_GPIO_WritePin(keypadColPort[col], keypadColPin[col], SET);

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



uint8_t calculator()
{
	static int i = 0;

	if(queue_empty() != 1)
	  {

		  key_value[i] = read_queue();
		  printf("%c\n", key_value[i]);
		  i++;

		  if(i == 4 && key_value[3] == '=')
		  {
			  switch(key_value[1])
			  {
			  case '+':
				  key_result = (key_value[0] - 0x30) + (key_value[2] - 0x30);
				  break;

			  case '-':
				  key_result = (key_value[0] - 0x30) - (key_value[2] - 0x30);
				  break;

			  case '*':
				  key_result = (key_value[0] - 0x30) * (key_value[2] - 0x30);
				  break;

			  case '/':
				  key_result = (key_value[0] - 0x30) / (key_value[2] - 0x30);
				  break;
			  }

			  printf("%c %c %c = %d \n", key_value[0], key_value[1], key_value[2], key_result);
			  i = 0;
		  }

	  }

	/*
	 * while (1)
  {

     if(TIM4_10ms_counter >= 6)
      {
         TIM4_10ms_counter = 0;
         unsigned char key_value = 0;

         key_value = keypadScan();
         if(key_value)
         {
            printf("%c\n", key_value);
            insert_queue(key_value); // circular queue에 넣는다.
         }
      }

     if(queue_empty() != 1)
     {
        if(rear1 - front1 == 4)
        {
           for(int i=0 ; i< 4; i++)
           {
              key_value1[i] = read_queue();
           }
           num1 = key_value1[0] - 0x30;
           num2 = key_value1[2] - 0x30;

           if(key_value1[3] == '=')
           {
              switch(key_value1[1])
              {
              case '+':
                 printf("%d + %d = %d\n", num1, num2, num1 + num2);
                 break;
              case '-':
                 printf("%d - %d = %d\n", num1, num2, num1 - num2);
                 break;
              case '*':
                 printf("%d * %d = %d\n", num1, num2, num1 * num2);
                 break;
              case '/':
                 printf("%d / %d = %d\n", num1, num2, num1 / num2);
                 break;
              }
           }
      }
     }
	 */


}



