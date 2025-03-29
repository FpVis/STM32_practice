#include "DS1302.h"



#define GPIOA_START_ADDR 0X40020000

t_ds1302 ds_time;

extern volatile int TIM11_DS1302;



void ds1302_main(void);
void ds1302_init(void);
void ds1302_CLK(void);
void ds1302_gpio_init(void);
void ds1302_DataLine_Input(void);
void ds1302_DataLine_Output(void);

void ds1302_tx(uint8_t value);
void ds1302_rx(unsigned char *data);

void ds1302_write(uint8_t addr, uint8_t data);
unsigned char ds1302_read(uint8_t addr);

void ds1302_read_time(void);


unsigned char bcd2dec(unsigned char byte);
unsigned char dec2bcd(unsigned char byte);


//===========================================================//

void ds1302_main()
{



		char lcd_buff[20];

			if(TIM11_DS1302 >= 1000)
			{
				TIM11_DS1302 = 0;
				ds1302_read_time();

				printf("*** %2d:%2d:%2d\n",
						ds_time.hours, ds_time.minutes, ds_time.seconds);

				move_cursor(0,0);
				lcd_string("CLOCK MODE       ");


				move_cursor(1,0);
				sprintf(lcd_buff, "%dH %dM %dS     ", ds_time.hours, ds_time.minutes, ds_time.seconds);
				lcd_string(lcd_buff);

			}


}



//===========================================================//


void ds1302_init(void)
{

	ds1302_write(ADDR_SECONDS, ds_time.seconds);
	ds1302_write(ADDR_MIN, ds_time.minutes);
	ds1302_write(ADDR_HOUR, ds_time.hours);

}


void ds1302_gpio_init(void)
{
	//HAL_GPIO_WritePin(CLK_DS1302_GPIO_Port, CLK_DS1302_Pin, GPIO_PIN_RESET);
	 *(unsigned int  *)0x40020014 &= ~(1 << 12);
	//HAL_GPIO_WritePin(IO_DS1302_GPIO_Port, IO_DS1302_Pin, GPIO_PIN_RESET);
	 *(unsigned int  *)0x40020014 &= ~(1 << 11);
	//HAL_GPIO_WritePin(CE_DS1302_GPIO_Port, CE_DS1302_Pin, GPIO_PIN_RESET);
	 *(unsigned int  *)0x40020014 &= ~(1 << 10);

}

void ds1302_CLK(void)
{
	HAL_GPIO_WritePin(CLK_DS1302_GPIO_Port, CLK_DS1302_Pin, GPIO_PIN_SET);
	// *(unsigned int  *)0x40020014 |= 1 << 12;
	HAL_GPIO_WritePin(CLK_DS1302_GPIO_Port, CLK_DS1302_Pin, GPIO_PIN_RESET);
	// *(unsigned int  *)0x40020014 &= ~(1 << 12);
}


void ds1302_DataLine_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = IO_DS1302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;            //Change Output to Input
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO_DS1302_GPIO_Port, &GPIO_InitStruct);

    return;
}


void ds1302_DataLine_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = IO_DS1302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            //Change Input to Output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IO_DS1302_GPIO_Port, &GPIO_InitStruct);

    return;
}


unsigned char bcd2dec(unsigned char byte)
{
	unsigned char high, low;

	high = (byte >> 4) * 10;
	low = byte & 0x0f;

	return(high + low);
}





unsigned char dec2bcd(unsigned char byte)
{
	unsigned char high, low;

	high = (byte / 10) << 4;
	low = byte % 10;

	return(high + low);
}


//===========================================================//


void ds1302_write(uint8_t addr, uint8_t data)
{
	/*
	 * 1. CE : LOW -> HIGH
	 * 2. ADDRESS TRANSMIT
	 * 3. DATA TRANSMIT
	 * 4. CE : HIGH -> LOW
	 */


	// 1. CE : LOW -> HIGH

	HAL_GPIO_WritePin(CE_DS1302_GPIO_Port, CE_DS1302_Pin, GPIO_PIN_SET);
	// *(unsigned int  *)0x40020014 |= 1 << 10;

	// 2. ADDRESS TRANSMIT

	ds1302_tx(addr);

	// 3. DATA TRANSMIT

	ds1302_tx(dec2bcd(data));

	// 4. CE : HIGH -> LOW

	HAL_GPIO_WritePin(CE_DS1302_GPIO_Port, CE_DS1302_Pin, GPIO_PIN_RESET);
	// *(unsigned int  *)0x40020014 &= ~(1 << 10);
}



void ds1302_tx(uint8_t value)
{
	//Initialize to Output
	ds1302_DataLine_Output();

	for(int i = 0; i < 8; i++)
	{
		if(value & (1 << i))
		{
			HAL_GPIO_WritePin(IO_DS1302_GPIO_Port, IO_DS1302_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(IO_DS1302_GPIO_Port, IO_DS1302_Pin, GPIO_PIN_RESET);
		}
		ds1302_CLK();
	}

}


//===========================================================//


void ds1302_read_time(void)
{
	ds_time.seconds = ds1302_read(ADDR_SECONDS);
	ds_time.minutes = ds1302_read(ADDR_MIN);
	ds_time.hours = ds1302_read(ADDR_HOUR);

}


unsigned char ds1302_read(uint8_t addr)
{
	/*
	 * 1. CE : LOW -> HIGH
	 * 2. ADDRESS TRANSMIT
	 * 3. DATA RECEIVE
	 * 4. CE : HIGH -> LOW
	 * 5. DATA OUT
	 */

	unsigned char data_8bit = 0;

	// 1. CE : LOW -> HIGH

		HAL_GPIO_WritePin(CE_DS1302_GPIO_Port, CE_DS1302_Pin, GPIO_PIN_SET);

	// 2. ADDRESS TRANSMIT

		ds1302_tx(addr + 0x01);

	// 3. DATA RECEIVE

		ds1302_rx(&data_8bit);

	// 4. CE : HIGH -> LOW

		HAL_GPIO_WritePin(CE_DS1302_GPIO_Port, CE_DS1302_Pin, GPIO_PIN_RESET);

	// 5. DATA OUT

		return(bcd2dec(data_8bit));
}



void ds1302_rx(unsigned char *data)
{
	//Convert INPUT TO OUTPUT
	ds1302_DataLine_Input();

	unsigned char temp = 0;


	for(int i = 0; i < 8; i++)
	{
		if(HAL_GPIO_ReadPin(IO_DS1302_GPIO_Port, IO_DS1302_Pin))
		{
			temp |= 1 << i;
		}

		if(i != 7 )
			ds1302_CLK();

	}

	*data = temp;

}

