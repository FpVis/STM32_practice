#include "button.h"
#include "stack.h"

unsigned char button_status[BUTTON_NUMBER]={
      BUTTON_RELEASE,BUTTON_RELEASE,BUTTON_RELEASE,BUTTON_RELEASE
};

extern void led_main(void);
extern void led_left_on(void);
extern void led_right_on(void);
extern void on_led(void);
extern void off_led(void);
extern void led_left_keepon(void);
extern void led_right_keepon(void);
extern void flower_on(void);
extern void flower_off(void);
extern uint8_t clock_mode;

extern int running;
extern lcd_init();
extern void lcd_set_cur(int row, int col);
extern uint8_t ds1302_run;
uint8_t hour_reset_button=0;
uint8_t min_reset_button=0;
uint8_t sec_reset_button=0;
uint8_t time_reset_run=0;


extern volatile int input_source;
extern uint8_t error;

void button_check(void)
{


   if(get_button(GPIOC,GPIO_PIN_5,BUTTON0)==BUTTON_PRESS)
   {
      hour_reset_button=1;
   }
   if(get_button(GPIOC,GPIO_PIN_6,BUTTON1)==BUTTON_PRESS)
   {
      min_reset_button=1;
   }

   if(get_button(GPIOC,GPIO_PIN_8,BUTTON2)==BUTTON_PRESS)
   {
      if(time_reset_run==0)
      {
      printf("mode change\n");
      clock_mode= !clock_mode;
      lcd_init();
      lcd_set_cur(0,0);
      }
      if(time_reset_run==1)
      {
         sec_reset_button=1;
      }

   }

   if(get_button(GPIOC,GPIO_PIN_9,BUTTON3)==BUTTON_PRESS)
   {
      printf("time set\n");
      //time_reset_run=1;
      //ds1302_run=1;
      lcd_init();
      lcd_set_cur(0,0);
   }

}
//버튼을 눌렀다 떼면 0 BUTTON_PRESS를 리턴
int get_button(GPIO_TypeDef *GPIO,uint16_t GPIO_Pin,int button_number){
   int state;

   state = HAL_GPIO_ReadPin(GPIO, GPIO_Pin); // active 0 inactive 1

   if(state == BUTTON_PRESS && button_status[button_number] == BUTTON_RELEASE){
      HAL_Delay(60);
      button_status[button_number]=BUTTON_PRESS;
      return BUTTON_RELEASE; // 아직 완전히 눌려지지 않은 상태

   }
   else if(button_status[button_number] == BUTTON_PRESS && state == BUTTON_RELEASE){
      //버튼이 이전에는 눌려진 상태이나 지금은 떼어진 상태일때
      button_status[button_number]=BUTTON_RELEASE;
      HAL_Delay(60);
      return BUTTON_PRESS;
   }

   return BUTTON_RELEASE;
}
