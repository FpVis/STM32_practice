#include "led.h"


void led_main(void);
void led_left_on(void);
void led_right_on(void);
void on_led(void);
void off_led(void);
void led_left_keepon(void);
void led_right_keepon(void);
void flower_on(void);
void flower_off(void);
void all_off(void);

int running=0;

int func_index=0;
void(*funcp[])() = {
		on_led,  //0
		off_led,  //1
		led_left_on,  //2
		led_right_on,  //3
		flower_on,  //4
		flower_off,  //5
		led_left_keepon,  //6
		led_right_keepon  //7
};



//none os 또는 loop monitor 방식
extern void button_check(void);
extern void pc_command_processing(void);
extern volatile int TIM11_1ms_counter;
void printf_num(void);

void led_main(void)
{
	while(1)
	{
		button_check();
		pc_command_processing();
	}
}

void flower_on(void){


#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
		TIM11_1ms_counter=0;

	if(i<4){
	HAL_GPIO_WritePin(GPIOB,0x08>>i | 0x10<<i,1);
	}

	i++;
   if(i>4)
   {i=0;
   HAL_GPIO_WritePin(GPIOB, 0xff, 0);
   }

	}

#else
	static int i=0;
	if(i==0){
			all_off();
		}
	if(i<4){
	HAL_GPIO_WritePin(GPIOB,0x08>>i,1);
	HAL_GPIO_WritePin(GPIOB,0x10<<i,1);
	HAL_Delay(100);
	}
    i++;
   if(i>=4)
   {i=0;
   HAL_GPIO_WritePin(GPIOB, 0xff, 0);
   HAL_Delay(100);
   }
#endif
/*  ------------------org-----------------
	for(int i=0;i<4;i++){
		HAL_GPIO_WritePin(GPIOB,0x08>>i,1);
		HAL_GPIO_WritePin(GPIOB,0x10<<i,1);
		//HAL_GPIO_WritePin(GPIOB,0x08>>i | 0x10<<i,1);
		HAL_Delay(200);
	}
*/

}

void flower_off(void){

#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
		TIM11_1ms_counter=0;
		if(i<4){
			HAL_GPIO_WritePin(GPIOB,0x80>>i |0x01<<i,0);
		}
		i++;
	if(i>4){
		i=0;
		HAL_GPIO_WritePin(GPIOB, 0xff, 1);
		}
	}

#else
	if(i<4){
		HAL_GPIO_WritePin(GPIOB,0x80>>i,0);
		HAL_GPIO_WritePin(GPIOB,0x01<<i,0);
		HAL_Delay(100);
		}
	    i++;
	   if(i>=4)
	   {i=0;
	   HAL_GPIO_WritePin(GPIOB, 0xff, 1);
	   HAL_Delay(100);
	   }
//	for(int i=0;i<4;i++){
//			HAL_GPIO_WritePin(GPIOB,0x80>>i,0);
//			HAL_GPIO_WritePin(GPIOB,0x01<<i,0);
//			//HAL_GPIO_WritePin(GPIOB,0x80>>i | 0x01<<i,0);
//			HAL_Delay(200);
//	}
#endif
}

void led_left_keepon(void){

#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
			TIM11_1ms_counter=0;
	if(i==0){
			all_off();
		}
	if(i<8){
		HAL_GPIO_WritePin(GPIOB,0x01<<i,1);
			}
	i++;
	if(i>8)
		   {i=0;
		   HAL_GPIO_WritePin(GPIOB,0xff,0);
		   }
	}
#else
	for(int i=0;i<8;i++){
			HAL_GPIO_WritePin(GPIOB,0x01<<i,1);
			HAL_Delay(200);
		}
	HAL_GPIO_WritePin(GPIOB,0xff,0);
#endif
}


void led_right_keepon(void){

#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
			TIM11_1ms_counter=0;
	if(i==0){
			all_off();
		}
	if(i<8){
		HAL_GPIO_WritePin(GPIOB,0x80>>i,1);
			HAL_Delay(100);
			}
	i++;
	if(i>8)
		   {i=0;
		   HAL_GPIO_WritePin(GPIOB,0xff,0);
		   HAL_Delay(100);
		   }
	}
#else
	for(int i=0;i<8;i++){
			HAL_GPIO_WritePin(GPIOB,0x80>>i,1);
			HAL_Delay(200);
		}
	HAL_GPIO_WritePin(GPIOB,0xff,0);
#endif
}
void led_left_on(void){
#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
		TIM11_1ms_counter=0;
		if(i==0){
			all_off();
		}
		if(i<8){
			HAL_GPIO_WritePin(GPIOB,0xff,0);
			HAL_GPIO_WritePin(GPIOB,0x01<<i,1);
			}
		i++;
		if(i>8){
			i=0;
			HAL_GPIO_WritePin(GPIOB,0xff,0);
		   }
	}
#else
	for(int i=0;i<8;i++){
		HAL_GPIO_WritePin(GPIOB,0xff,0);
		HAL_GPIO_WritePin(GPIOB,0x01<<i,1);
		HAL_Delay(200);
	}
	HAL_GPIO_WritePin(GPIOB,0xff,0);
	HAL_Delay(200);
#endif
}

void led_right_on(void){
#if 1

	static int i=0;

	if(TIM11_1ms_counter >100){
			TIM11_1ms_counter=0;
	if(i==0){
			all_off();
		}
	if(i<8){
		HAL_GPIO_WritePin(GPIOB,0xff,0);
		HAL_GPIO_WritePin(GPIOB,0x80>>i,1);
	}
	i++;
	if(i>8)
		   {i=0;
		   HAL_GPIO_WritePin(GPIOB,0xff,0);
		   }
	}
#else
	for(int i=0;i<8;i++){
		HAL_GPIO_WritePin(GPIOB,0xff,0);
		HAL_GPIO_WritePin(GPIOB,0x80>>i,1);
		HAL_Delay(200);
	}
	HAL_GPIO_WritePin(GPIOB,0xff,0);
		HAL_Delay(200);
#endif
}
void all_off(void){
	HAL_GPIO_WritePin(GPIOB,0xff,0);
}

//bit shift 연산자를 이용해 3번 bit를 set
void on_led(void){
	HAL_GPIO_WritePin(GPIOB,0xff,1);
}
//bit shift 연산자를 이용해 3번 bit를 reset
void off_led(void){
	HAL_GPIO_WritePin(GPIOB,0xff,0);
}
