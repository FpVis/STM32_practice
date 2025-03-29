#include "main.h"


enum state_t{OK, TIME_OUT, VALUE_ERROR, TRANS_ERROR};


void dht11_main(void);
void DHT11_DataLine_Input(void);
void DHT11_DataLine_Output(void);
extern void delay_us(int us);

extern volatile int TIM11_DHT11_counter;


void DHT11_DataLine_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;            //Change Output to Input
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

    return;
}


void DHT11_DataLine_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            //Change Input to Output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

    return;
}


void dht11_main(void)
{
	if(TIM11_DHT11_counter >= 1000)
	{

		TIM11_DHT11_counter = 0;
		int data[5] = {0, };	// humi(습도)2 BYTE + temp(온도)2 BYTE + checksum 1BYTE
		int us_count = 0;	//	FOR TIME CONTROL

		uint8_t state = OK;


			for(int i = 0; i < 5; i++)
			{
				data[i] = 0;	//	data 버퍼 초기화. memset() 함수를 응용할 수도 있다.
			}

			//========= STEP1 : Request to DHT_11 (Handshake)


						state = OK;
						DHT11_DataLine_Output();
						HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
						HAL_Delay(100);



						HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
						HAL_Delay(20);

			HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
			DHT11_DataLine_Input();
			delay_us(1);

			us_count = 0;
			while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal LOW
			{
				delay_us(2);
				us_count += 2;
				if(us_count > 50)
				{
					state = TIME_OUT;	//	 TIME_OUT Exception Case
					printf("HANDSHAKE_TIMEOUT_01\n");
					break;
				}

			}

			//========= STEP2 : Response from DHT_11 (Handshake)

			if (state == OK)
			{
				us_count = 0;
				while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal HIGH
					{
						delay_us(2);
						us_count += 2;
						if(us_count > 100)
						{
							state = TIME_OUT;	//	 TIME_OUT Exception Case
							printf("HANDSHAKE_TIMEOUT_02\n");
							break;
						}

					}

			}



			if (state == OK)
			{
				us_count = 0;
				while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal LOW
					{
						delay_us(2);
						us_count += 2;
						if(us_count > 100)
						{
							state = TIME_OUT;	//	 TIME_OUT Exception Case
							printf("HANDSHAKE_TIMEOUT_03\n");
							break;
						}

					}
			}


			//========= STEP3 : Receive DATA from DHT_11
			// 습도 정수부 8BIT, 습도 소수부 8BIT, 온도 정수부 8BIT, 온도 소수부 8BIT, CHECK SUM 8BIT
			// 0 : 50us LOW --> 26 ~ 28us HIGH
			// 1 : 50us LOW --> 70us HIGH

			if (state == OK)
			{
				int pulse[8];	//	 1 BYTE (STORE 8 PULSE BIT)

				for(int i = 0; i < 5; i++)
				{
					for(int j = 7; j >= 0 ; j--)
					{
						us_count = 0;
						while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal HIGH
								{
									delay_us(2);
									us_count += 2;
									if(us_count > 70)
									{
										state = TIME_OUT;	//	 TIME_OUT Exception Case
										i = 5;
										j = -1;				//	Escape from for()
										printf("READ_TIMEOUT_01\n");
										break;
									}

								}


						if(state == OK)
						{
							us_count = 0;
							while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal LOW
								{
									delay_us(2);
									us_count += 2;
									if(us_count > 90)
									{
										state = TIME_OUT;	//	 TIME_OUT Exception Case
										i = 5;
										j = -1;				//	Escape from for()
										printf("READ_TIMEOUT_02\n");
										break;
									}

								}


							if(state == OK)
							{
								if(us_count < 40)		//	DATA IS 0
									pulse[j] = 0;

								else if(us_count > 40)	//	DATA IS 1
									pulse[j] = 1;

							}
						}
					}


					if(state == OK)
					{
						/*
						for(int k = 0; k < 8; k++)
						{
							data[i] = pulse[k] << k;
						}
						*/
						data[i] = (pulse[0] << 0) | (pulse[1] << 1) | (pulse[2] << 2) |
								  (pulse[3] << 3) | (pulse[4] << 4) | (pulse[5] << 5) |
								  (pulse[6] << 6) | (pulse[7] << 7);

					}
				}

				//========= STEP4 : Check sum

				if(state == OK)
				{
					if(data[4] != data[0] + data[1] + data[2] + data[3])
					state = VALUE_ERROR;
				}

				//========= STEP4-1 : CHECK FOR ANOTHER DATA
				delay_us(60);
				us_count = 0;

				while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))	//	Wait Until DHT Signal HIGH
				{
					delay_us(2);	//
					us_count += 2;
					if(us_count > 90)
					{
						break;
					}

				}

			}


			//========= STEP5 : Print Value
			if(state == OK)
			{
				printf("HUMI : %d.%d %\t\tTEMP : %d.%d C\n\n", data[0], data[1], data[2], data[3]);
			}
			if(state != OK)
			{
				printf("ERROR CODE : %d\n", state);
				printf(("DATA : %d %d %d %d %d\n\n"), data[0], data[1], data[2], data[3], data[4]);
			}

	}


}
