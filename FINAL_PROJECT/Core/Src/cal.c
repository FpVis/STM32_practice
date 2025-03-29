#include "main.h"
#include "stack.h"
#include "keypad.h"
#include "circularQueue.h"
#include "button.h"

t_Stack stack;
extern uint8_t error;

void sim_cal(void);
int isDigit(char ch);
int isOper(unsigned char ch);
int precedence(char op);
void infixToPostfix(char* infix, char* postfix);


extern void push(t_Stack* stack, int value);
extern int get_button(GPIO_TypeDef* GPIO, uint16_t GPIO_Pin, int button_number);
extern void insert_queue(unsigned char value);
extern void stack_init(t_Stack* stack);
extern rear1;
extern unsigned char queue[100];
extern lcd_init();
extern void lcd_send_string(char *str);
extern void lcd_set_cur(int row, int col);

char postfix[MAX];

int isOper(unsigned char ch)
{
	return ch == '+' || ch == '-' || ch == '*' || ch == '/' || ch == '(' || ch == ')'  ;
}

int isDigit(char ch)
{
   return ch >= '0' && ch <= '9';
}

int precedence(char op)
{
    switch (op)
    {
        case '+': return 1; // 덧셈의 우선순위: 1
        case '-': return 1; // 뺄셈의 우선순위: 1
        case '*': return 2; // 곱셈의 우선순위: 2
        case '/': return 2; // 나눗셈의 우선순위: 2
        default:  return 0; // 연산자가 아닌 경우
    }
}


void Calculator1(void)
{

	char result_buff[100]={};
    if(get_button(GPIOC,GPIO_PIN_5,BUTTON0) == BUTTON_PRESS)
    {
        insert_queue('(');  // 여는 괄호 '(' 입력
    }

    if(get_button(GPIOC,GPIO_PIN_6,BUTTON1) == BUTTON_PRESS)
    {
        insert_queue(')');  // 닫는 괄호 ')' 입력
    }

    if(get_button(GPIOC,GPIO_PIN_9,BUTTON3) == BUTTON_PRESS)
	{
    	lcd_init();
    	lcd_set_cur(0, 0);
		error=0;

	}

    if (queue_empty() != 1)  // 큐가 비어 있지 않으면
    {
        char key_value;

        key_value = read_queue();  // 큐에서 값 읽기
        sprintf(result_buff,"%c",key_value);
        lcd_send_string(result_buff);
        printf("%c\n", key_value);  // 읽은 값 출력

        if (key_value == '=')
        {
        	char postfix[MAX];
        	infixToPostfix(queue, postfix);
        	int result = evaluatePostfix(postfix);

        	if(error !=1){

                    printf("Result: %d\n", result);  // 계산 결과 출력
                    lcd_set_cur(1,0);
                    sprintf(result_buff,"%d",result);
                    lcd_send_string(result_buff);
        	}

                // 스택 안의 모든 값 출력

            // 큐와 스택 초기화
            queue_init();
            stack_init(&stack);
        }
        else if (isOper(key_value))  // 연산자 처리
        {
            push(&stack, key_value);  // 연산자를 스택에 넣음
        }
    }
}


void infixToPostfix(char* infix, char* postfix)
{
  stack_init(&stack);
  int j = 0;


  for (int i = 0; infix[i] != '\0'; i++)
  {
    char ch = infix[i];

    if (isDigit(ch))
    {

      while (isDigit(infix[i]))      //    숫자가 두자릿수 이상인 경우를 위해
      {
        postfix[j++] = infix[i++];      //      1. 숫자는 그대로 출력한다.
      }


      postfix[j++] = ' ';
      i--;  // 반복문에서 i++을 건너뛰지 않도록 조정
    }

    else if (ch == '(')         //      7. 여는 괄호는 스택에 그냥 추가한다.
    {
      push(&stack, ch);
    }

    else if (ch == ')')
    {
      while (!isEmpty(&stack) && peek(&stack) != '(')      //      9. 닫는 괄호는 여는 괄호가 나올 때까지 스택을 pop 하여 출력한다. 다 출력하고 난 뒤 괄호들은 버린다.
        postfix[j++] = pop(&stack);

      pop(&stack);  // 9. 다 출력하고 난 뒤 괄호들은 버린다.
    }

    else if (isOper(ch))
    {
      while (!isEmpty(&stack) && precedence(peek(&stack)) >= precedence(ch))      //      2. 만약 스택이 비어있다면 연산자를 그냥 스택에 넣는다.
        postfix[j++] = pop(&stack);         //         4. (스택의 top에 있는 연산자의 우선순위 >= 현재 연산자의 우선순위) 이면 2번 혹은 3번 상황이 될 때까지 pop 하여 출력하고 연산자를 스택에 넣는다.


      push(&stack, ch);      //      3. (스택의 top에 있는 연산자의 우선순위 < 현재 연산자의 우선순위) 이면 현재 연산자를 그냥 스택에 넣는다.

    }
  }


  while (!isEmpty(&stack))
  {
    postfix[j++] = pop(&stack);      //      5. 모든 수식을 다 사용했다면 스택이 빌 때까지 pop하여 출력한다.
  }
  postfix[j] = '\0';
}



int evaluatePostfix(char* postfix) {
	stack_init(&stack);
  int i = 0;

  while (postfix[i] != '\0')      //      postfix 안의 값이 모두 출력될때까지 실행.
  {
    if (isDigit(postfix[i]))
    {
      int num_buffer = 0;


      while (isDigit(postfix[i]))      //    숫자가 두자릿수 이상인 경우를 위해
      {
        num_buffer = num_buffer * 10 + (postfix[i] - '0');      //      문자 '0'을 빼서 문자 형태의 숫자('0' ~ '9')를 정수로 변환.
        i++;                                       //      현재 num의 값을 10배 늘려 자릿수를 앞으로 한 칸 밀고, postfix[i]의 값을 더해 새로운 자리 값을 추가. (Ex. num이 2일 때 postfix[i]가 '0'이라면 num * 10 + 0이 되어 num이 20이 됨)
      }
      push(&stack, num_buffer);      //      1. 숫자는 스택에 그냥 추가한다.

    }

    else if (isOper(postfix[i]))      //      2. 연산자가 나오면 숫자 2개를 pop 해서 계산한다.
    {

      int val2 = pop(&stack);
      int val1 = pop(&stack);      //      3. 이때 먼저 pop 되는 숫자가 두 번째 값, 나중에 pop되는 숫자가 첫 번째 값으로 하여 계산해야 한다.

      switch (postfix[i])
      {
      case '+': push(&stack, val1 + val2); break;
      case '-': push(&stack, val1 - val2); break;
      case '*': push(&stack, val1 * val2); break;
      case '/': push(&stack, val1 / val2); break;      //   3. 계산한 값은 다시 스택에 넣는다.
      }
      i++;

    }

    else

    {
      i++;  // 공백 건너뛰기
    }


  }

  return pop(&stack);      //      최종적으로 계산된 값을 출력

}















