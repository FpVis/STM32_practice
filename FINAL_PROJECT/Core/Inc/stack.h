#include "main.h"

#define MAX 100

// t_Stack 구조체 정의
typedef struct {
   int top;
   int items[MAX];
} t_Stack;

// 함수 선언
int isEmpty(t_Stack* stack);
int isFull(t_Stack* stack);
void push(t_Stack* stack, int value);
int pop(t_Stack* stack);
void stack_init(t_Stack* stack);
