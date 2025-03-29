#include "main.h"

#define BUTTON_NUMBER 4
#define BUTTON0 0  //PC5
#define BUTTON1 1  //PC6
#define BUTTON2 2  //PC8
#define BUTTON3 3  //PC9
#define BUTTON_default 4
//------pull-up 설정 ---------
#define BUTTON_RELEASE 1 //버튼을 땐 상태
#define BUTTON_PRESS 0   //버튼을 누른 상태
#define INPUT_BUTTON 0
#define INPUT_UART 1


int get_button(GPIO_TypeDef* GPIO, uint16_t GPIO_Pin, int button_number);
void button_check(void);
