#include "main.h"   // HAL & GPIO

#define BUTTON_NUMBER 4
#define BUTTON0  0  //PC5
#define BUTTON1  1  // PC6
#define BUTTON2  2  // PC8
#define BUTTON3  3  // PC9
//-------- pull-up 설정 ------
#define BUTTON_RELEASE  1   // 버튼을 뗀 상태 high 1
#define BUTTON_PRESS  0     // 버튼을 누른 상태 low 0 active-low

int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin, int button_number);
void button_check(void);
