
#include "button.h"

unsigned char button_status[BUTTON_NUMBER] = {
	BUTTON_RELEASE, BUTTON_RELEASE, BUTTON_RELEASE, BUTTON_RELEASE
};

extern void led_main(void);
extern void led_left_on(void);
extern void led_right_on(void);
extern void led_left_keepon(void);
extern void led_right_keepon(void);
extern void led_flower_on(void);
extern void led_flower_off(void);

extern void on_led(int led_number);
extern void off_led(int led_number);
extern void on_led(int led_number);
extern void off_led(int led_number);
extern void (*funcp[])();

extern int func_index;


void button_check(void)
{
	static uint8_t demo_led=0;  // uint8 --> unsigned char
	static int8_t btn=0;
	           // 지역변수지만 변수 type앞에 static을 선언 하면 전역변수 처럼 동작
	// 1버튼을 1번 눌렀다 떼면 demo led가 on / off 되도록 구현
	if (get_button(GPIOC, GPIO_PIN_5, BUTTON0) == BUTTON_PRESS)
	{
		btn = !btn;
		func_index = (btn == 0) ? 0 : 1;
	}
	if (get_button(GPIOC, GPIO_PIN_6, BUTTON1) == BUTTON_PRESS)
	{
		btn = !btn;
		func_index = (btn == 0) ? 2 : 3;
	}
	if (get_button(GPIOC, GPIO_PIN_8, BUTTON2) == BUTTON_PRESS)
	{
		btn = !btn;
		func_index = (btn == 0) ? 4 : 5;
	}
	if (get_button(GPIOC, GPIO_PIN_9, BUTTON3) == BUTTON_PRESS)
	{
		btn = !btn;
		func_index = (btn == 0) ? 6 : 7;
	}

	funcp[func_index]();
}

// 버튼을 눌렀다 떼면 0 BUTTON_PRESS을 리턴
int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin, int button_number)
{
	int state;

	state = HAL_GPIO_ReadPin(GPIO, GPIO_Pin);  // active:0 inactive:1
    // 처음 눌려진 상태 냐 ?
	if (state == BUTTON_PRESS && button_status[button_number] == BUTTON_RELEASE)
	{
		HAL_Delay(60);   // 노이즈가 지나가기를 기다린다.
		button_status[button_number]=BUTTON_PRESS; // 처음 눌려진 상태가 아님을 알림
		return BUTTON_RELEASE;  // 아직은 완전히 눌려진 상태가 아니다.
	}
	else if (button_status[button_number] == BUTTON_PRESS && state == BUTTON_RELEASE)
	{  // 버튼이 이전에는 울려진 상태이나 지금은 떼어진 상태 이면
		button_status[button_number]=BUTTON_RELEASE;  // 다음 버튼 상태를 체크하기 위해서 초기화
		HAL_Delay(60);   // 노이즈가 지나가기를 기다린다.
		return BUTTON_PRESS;   // 완전히 눌렀다 떼어진 안정된 상태로 인정
	}
	return BUTTON_RELEASE;   // 눌려진 상태가 아니다.
}
