#include "main.h"

#define BUTTON_NUMBER 4

#define BUTTON0 0
#define BUTTON1 1
#define BUTTON2 2
#define BUTTON3 3

#define BUTTON_RELEASE 1
#define BUTTON_PRESS 0

void button_check(void);
int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin, int button_number);
