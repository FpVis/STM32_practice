#include "stack.h"
extern lcd_init();
extern void lcd_send_string(char *str);
extern void lcd_set_cur(int row, int col);
extern queue_init();
uint8_t error =0;
// 스택이 비었는지 확인하는 함수
int isEmpty(t_Stack* stack)
{
   return stack->top == -1;
}

// 스택이 가득 찼는지 확인하는 함수
int isFull(t_Stack* stack)
{
   return stack->top == MAX - 1;  // Stack이 가득 찼는지 확인
}

// 값 추가하는 함수
void push(t_Stack* stack, int value)
{
   if (isFull(stack))
   {
		printf("Stack overflow\n");
		lcd_set_cur(1,0);
		lcd_send_string("wrong calculate");
		error=1;

   }
   stack->items[++(stack->top)] = value;
}

// 값 제거하는 함수
int pop(t_Stack* stack)
{
   if (isEmpty(stack))
   {
		printf("Stack underflow\n");
		lcd_set_cur(1,0);
		lcd_send_string("wrong calculate");
		error=1;

   }
   return stack->items[(stack->top)--];
}

// 스택 초기화 함수
void stack_init(t_Stack* stack)
{
   stack->top = -1;  // 스택 초기화
}

int peek(t_Stack* stack)
{
  return stack->items[stack->top];
}

