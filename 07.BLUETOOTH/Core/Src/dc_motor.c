#include "main.h"
#include "button.h"

extern TIM_HandleTypeDef htim3;


uint8_t pwm_start_flag = 0;
uint16_t pwm_value = 0;

void dc_motor_pwm_control(void);










void dc_motor_pwm_control(void){

	//	START / STOP
	if(get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)
	{

		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

		if(!pwm_start_flag)
		{

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

		}
		else
		{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		}

		pwm_start_flag = !pwm_start_flag;

	}

	//	SPEED UP - PWM 값을 증가 시킴.
	if(get_button(GPIOC, GPIO_PIN_6, BUTTON1) == BUTTON_PRESS)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		pwm_value = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
		pwm_value += 10;

		if(pwm_value > 100)
			pwm_value = 100;

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);

	}

	//	SPEED DOWN
	if(get_button(GPIOC, GPIO_PIN_8, BUTTON2) == BUTTON_PRESS)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		pwm_value = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
		pwm_value -= 10;

		if(pwm_value < 70)
			pwm_value = 70;

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
	}


}

/*
 __HAL_TIM_GET_COMPARE는 STM32의 HAL 라이브러리에서 사용되는 매크로로, 특정 타이머 채널의 비교값(Capture/Compare Register, CCR) 을 읽어오는 데 사용됩니다.

정의
c
코드 복사
__HAL_TIM_GET_COMPARE(__HANDLE__, __CHANNEL__)
__HANDLE__: 타이머의 핸들러 구조체(TIM_HandleTypeDef).
__CHANNEL__: 비교값을 읽어올 타이머 채널. 예를 들어, TIM_CHANNEL_1, TIM_CHANNEL_2 등이 있습니다.
주요 역할
이 매크로는 타이머의 특정 채널에 설정된 캡처/비교 레지스터(CCR) 값을 반환합니다.

PWM 출력: 현재 PWM 신호의 듀티 사이클을 확인할 때 유용합니다.
입력 캡처: 입력 신호가 특정 기준과 비교되어 타이머 값이 저장된 경우 이를 읽을 수 있습니다.
출력 비교: 타이머가 지정된 비교값에 도달했는지 확인하는 데 사용됩니다.
사용 예시
1. PWM 모드에서 듀티 사이클 확인
c
코드 복사
uint32_t duty_cycle = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
위 코드는 TIM2의 채널 1에 설정된 비교값(PWM 듀티 사이클)을 읽어옵니다.

2. 입력 캡처 모드에서 캡처된 값 확인
c
코드 복사
uint32_t captured_value = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
TIM3의 채널 2에서 입력 신호에 의해 캡처된 값을 읽어옵니다.

 */
