/*
 * dcmotor.c
 */


#include "main.h"
#include "button.h"
#include "i2c_lcd.h"


uint8_t auto_mode_state=0;
uint8_t mode_change=0;

extern TIM_HandleTypeDef htim3;
extern volatile int TIM11_1ms_counter;

void mode_check();
void drive_car_main();
void auto_drive(void);
void manual_mode_run(void);

extern int dis_left;
extern int dis_right;
extern int dis_center;
extern TIM_HandleTypeDef htim1;

extern void ultrasonic_processing(void);
/*
  1. LEFT MOTOR
     PC.6 : IN1
	 PC.7 : IN2
  2. RIGHT MOTOR
     PC.8 : IN3
	 PC.9 : IN4

	 IN1.IN3  IN2.IN4
	 ======= ========
	   0        1  : 역회전
	   1        0  : 정회전
	   1        1  : STOP
*/

void drive_car_main()
{
    while (1)
    {

		mode_check();            // button1 check
		if (auto_mode_state)
		{
			auto_drive();
		}
		else if(auto_mode_state==0)
		{
			manual_mode_run();       // bluetooth car command  run
		}
    }

}

void mode_check()
{
	if (get_button(BUTTON0_GPIO_Port, BUTTON0_Pin, BUTTON0) == BUTTON_PRESS)
	{
		auto_mode_state = !auto_mode_state;
		move_cursor(0,0);
		if (auto_mode_state)
		{
			lcd_string("AUTO Mode       ");
		}
		else
		{
			lcd_string("Manual Mode    ");
			move_cursor(1,0);
			lcd_string("                   ");
		}
	}
}

// 자율주행 프로그램을 이곳에 programming 한다.
void auto_drive(void)
{
	if(mode_change==0)
	{
		stop();
		mode_change=1;
	}
	ultrasonic_processing();
	 dis_left, dis_center, dis_right;

	 if (dis_left == 0 || dis_center == 0 || dis_right == 0 )
	 {
		 HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
		 HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
		 HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_4);

	     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	     HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	     HAL_Delay(100);
	 }


	 if (dis_left<5 || dis_center<5 || dis_right<5 )
	 {
	 	backward(60);
	 	HAL_Delay(100);
	 }

	 if (dis_left<15 || dis_center<15 || dis_right<15 )
	 {
		 if(dis_left>dis_right)
		{
			turn_left(70);
		}

		else if(dis_left<dis_right)
		{
			turn_right(70);
		}
	 }

	 else if (dis_left<25 || dis_center<25 || dis_right<25 )
	 {
		 if(dis_left>dis_right)
		{
			turn_left_foward(60,30);
		}

		else if(dis_left<dis_right)
		{
			turn_right_foward(60,30);
		}
	 }

	 else if (dis_left<35 || dis_center<35 || dis_right<35 )
	 {
		 if(dis_left>dis_right)
		{
			turn_left_foward(60,50);
		}

		else if(dis_left<dis_right)
		{
			turn_right_foward(60,50);
		}
	 }

	 else
	{
		forward(50);
	}




}

extern volatile uint8_t rx_btdata;   // 2. BT로 부터 1byte의 INT가 들어오면 저장 하는 변수

void manual_mode_run(void)
{
	if(mode_change == 1)
	{
		stop();
		mode_change==0;
	}

	if (rx_btdata == 'F'|| rx_btdata == 'f')
	{
		move_cursor(1,0);
		lcd_string("Forward    ");
	}
	else if (rx_btdata == 'B'||rx_btdata == 'b')
	{
		move_cursor(1,0);
		lcd_string("Backward    ");
	}
	else if (rx_btdata == 'L'||rx_btdata == 'l')
	{
		move_cursor(1,0);
		lcd_string("Turn Left    ");
	}
	else if (rx_btdata == 'R'||rx_btdata == 'r')
	{
		move_cursor(1,0);
		lcd_string("Turn Right   ");
	}


	switch(rx_btdata)
	{
		case 'F':
		case 'f':
		forward(100);
		break;
		case 'B':
		case 'b':
		backward(100);
		break;
		case 'L':
		case 'l':
		turn_left(100);
		break;
		case 'R':
		case 'r':
		turn_right(100);
		break;
		case 'S':
		case 's':
		stop();
		break;
		default:
		break;
	}
}

void forward(int speed)
{
	all_foward();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);   // left speed

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);   //  right speed
}

void backward(int speed)
{
	all_backward();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);   // left speed

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);   //  right speed
}

void turn_left(int speed)
{
	all_foward();

	left_speed(0);
	right_speed(speed);   //  PWM 출력 right
}

void turn_right(int speed)
{
	all_foward();

	left_speed(speed); //  PWM 출력	  left
	right_speed(0);    //  PWM 출력 right
}

void turn_left_foward(int speed, int speed2)
{
	all_foward();

	left_speed(speed2);
	right_speed(speed);   //  PWM 출력 right
}

void turn_right_foward(int speed, int speed2)
{
	all_foward();

	left_speed(speed); //  PWM 출력	  left
	right_speed(speed2);    //  PWM 출력 right
}

void turn_left_backward(int speed)
{
	all_backward();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);   // left speed

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);   //  right speed
}

void turn_right_backward(int speed)
{
	all_backward();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);   // left speed

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);   //  right speed
}

void stop()
{
	all_stop();

	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);

}


void left_speed(uint16_t speed)
{
	if (speed >= 100) speed = 100;
	else if (speed < 0) speed = 0;

	if (speed == 0)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	}
	else
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
	}

}

void right_speed(uint16_t speed)
{
	if (speed >= 100) speed = 100;
	else if (speed < 0) speed = 0;

	if (speed == 0)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	}
	else
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
	}
}

void left_forward()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void right_forward()
{
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void all_foward()
{
	 left_forward();
	 right_forward();
}

void left_backward()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

}

void right_backward()
{
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}

void all_backward()
{
	left_backward();
	right_backward();
}

void left_stop()
{
	left_speed(0);
}

void right_stop()
{
	right_speed(0);
}

void all_stop()
{
	 left_stop();
	 right_stop();
}

