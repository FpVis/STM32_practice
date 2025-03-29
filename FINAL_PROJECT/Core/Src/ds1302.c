#include "ds1302.h"
#include "circularQueue.h"
#include "button.h"
#include <stdio.h>
#include <stdlib.h>

#define GPIOA_START_ADDR 0x40020000

t_ds1302 ds_time;
uint8_t clock_mode=0;
uint8_t input_mode=0;
uint8_t ds1302_run=0;
uint16_t result_num = 0;

int select_ = 0;
int id_check=0;
int password =0;
int save_id;
unsigned char result[6];

extern t_st st[8];
extern unsigned char queue[QUEUE_MAX];




void ds1302_main();

void ds1302_read_time(void);
void ds1302_read_date(void);

void time_display(void);
extern void button_check(void);
extern void keypad_main();

extern void Calculator1(void);

extern void lcd_init(void);
extern void lcd_send_string(char *str);
extern void lcd_set_cur(int row, int col);

unsigned char bcd2dec(unsigned char byte);
unsigned char dec2bcd(unsigned char byte);
void ds1302_gpio_init(void);
void ds1302_init(void);
void ds1302_write(uint8_t addr,uint8_t data);
void ds1302_tx(uint8_t value);
void ds1302_DataLine_Input(void);
void ds1302_clock(void);
void ds1302_DataLine_Output(void);
void ds1302_rx(unsigned char *data);
unsigned char ds1302_read(uint8_t addr);


void enter_input(t_list* list);

extern void servo_motor_main(void);

extern void queue_init(void);

extern HAL_StatusTypeDef flash_write_time(uint32_t *data32, int size);
extern HAL_StatusTypeDef flash_write(uint32_t *data32, int size);
extern void flash_ds1302();


extern char key_value;
extern volatile int TIM11_1ms_ds1302;
extern volatile int TIM11_1ms_counter;
extern int input_num;
extern int start;
extern t_list list;

extern uint8_t time_reset_run;
extern uint8_t hour_reset_button;
extern uint8_t min_reset_button;
extern uint8_t sec_reset_button;



void ds1302_main()

{


   unsigned char menu_select;
   char lcd_buff[20];
   static int demo_led = 0;
   static unsigned char result[100];
   ds1302_gpio_init();
   int MODE=0;
   flash_ds1302();

   //flash_ds1302();
   //ds1302_init(); //ds1302에 ds_time값을 write완료

      while(1)
      {

         button_check(); //버튼2를 눌러서 입퇴실 기능과 계산기 기능을 토글

            if(ds1302_run==0)
            {
            // 1. ds1302의 시간값을 읽고
            ds1302_read_time();
            // 2. ds1302의 날짜값을 읽고
            ds1302_read_date();
            }

            if(time_reset_run==1)
            {
               time_reset();
            }


         if(clock_mode==0) //입퇴실 기능
         {
            if(queue_empty() != 1)
            {
               key_value=read_queue();
               sprintf(lcd_buff,"%c",key_value);
               lcd_send_string(lcd_buff);

               if(key_value==' ')
               {
                  input_mode = !input_mode;
                  select_=0;  //menu_seclect
                  start=0; //
                  id_check=0;
               }
            }

            if(input_mode==0)  //키패드를 안눌렀을때
            {
               time_display(); // 시계 출력
            }

            else if(input_mode==1) //공백 버튼을 누르면
            {
               if(select_==0)
               {
                  if(!start) // 키패드 첫 입력때
                  {
                     start=1;
                     show_menu();
                  }

                  if(key_value=='1' || key_value=='2' ||key_value=='3')
                  {
                     switch (key_value)
                     {
                     case '1':
                        MODE=1;
                        select_=1;
                        lcd_init();
                        lcd_set_cur(0,0);
                        lcd_send_string("INPUT ID");
                        lcd_set_cur(1,0);
                        queue_init();
                        id_check=1;
                        break;
                     case '2':
                        MODE=2;
                        select_=1;
                        lcd_init();
                        lcd_set_cur(0,0);
                        lcd_send_string("INPUT ID");
                        lcd_set_cur(1,0);
                        queue_init();
                        id_check=1;
                        break;
                     case '3':
                        MODE=3;
                        select_=1;
                        ds1302_run=1;
                        time_reset_run=1;

                        break;
                       }
                  }

               }

               else if(select_ ==1 && queue[4]=='=')
               {
                  if(password==0)
                  {
                     if(MODE==1){
                        enter_input(&list);
                     }
                     else if(MODE==2){
                        show(&list);
                     }
                     else if(MODE==3){

                     }
                  }
               }
            }
         }

         else if(clock_mode==1) //
         {
           Calculator1();
         }
      }
}




// uint8_t Year
// 예) 24년의 Year에 저장된 data format
//  7654 3210
//  0010 0100
//    2   4
//  ===> 24
unsigned char bcd2dec(unsigned char byte)
{
   unsigned char high, low;

   low = byte & 0x0f;
   high = (byte >> 4) * 10;

   return (high+low);
}

//          10진수를       bcd로 변환
// 예) 24 ( 00011000) --> 0010 0100
//STM32의 RTC에서 날짜와 시각정보를 읽어 오는 함수를 작성
unsigned char dec2bcd(unsigned char byte)
{
   unsigned char high, low;


   high = (byte / 10) << 4;
    low  = byte % 10;

   return (high+low);
}

void ds1302_gpio_init()
{
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 12);   //CLK_DS1302 Reset Write
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 11);   // IO_DS1302 Reset Write
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 10);   // CE_DS1302 Reset Write
}

void ds1302_init(void)
{
   ds1302_write(ADDR_SECONDS,ds_time.seconds);
   ds1302_write(ADDR_MINUTES,ds_time.minutes);
   ds1302_write(ADDR_HOURS,ds_time.hours);
   ds1302_write(ADDR_DATE,ds_time.date);
   ds1302_write(ADDR_MONTH,ds_time.month);
   ds1302_write(ADDR_DAYOFWEEK,ds_time.dayofweek);
   ds1302_write(ADDR_YEAR,ds_time.year);
}

void ds1302_write(uint8_t addr,uint8_t data)
{
   //1 . CE low --> high
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) |= 1 << 10;      // //CE_DS1302 set Write
   //2 . addr 전송
   ds1302_tx(addr);
   //3 . data 전송
   ds1302_tx(dec2bcd(data));
   //4. CE high --> low
      *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 10);   // CE_DS1302 Reset Write
}

void ds1302_tx(uint8_t value)
{
   ds1302_DataLine_Output();
   // addr 초를 write
   // 80h  M       L
   //      1000 0000
   //      0000 0001        & operation
   //    ===============================
   //      0000 0000

   // 80h  1000 0000
   //      1000 0000    &
   // =====================
   //      1000 0000

   for(int i=0;i<8;i++)
   {
      if(value & (1<<i))   //bit가 1인 경우
      {
            *(unsigned int *) (GPIOA_START_ADDR + 0x14) |= 1 << 11;      // //IO_DS1302 set Write
      }
      else // bit가 0 인 겨우
      {
            *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 11);   // IO_DS1302 Reset Write
      }
      ds1302_clock();
   }
}
void ds1302_DataLine_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = IO_DS1302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;            //Change Output to Input
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO_DS1302_GPIO_Port, &GPIO_InitStruct);

    return;
}

void ds1302_clock(void)
{
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) |= 1 << 12;      // //CLK_DS1302 set Write
   *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 12);   //CLK_DS1302 Reset Write
}
void ds1302_DataLine_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = IO_DS1302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            //Change Input to Output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // LOW : 2MHz  HIGH : 25~100MHz
  HAL_GPIO_Init(IO_DS1302_GPIO_Port, &GPIO_InitStruct);

    return;
}

void ds1302_rx(unsigned char *data)
{
   unsigned char temp = 0;

   ds1302_DataLine_Input(); //입력 모드로 전환

   // ds1302로부터 넘어온 data를 LSB로부터 받아 들인다
   for(int i=0;i<8;i++)
   {
      //1 bit 읽어 들인다.
      //
         if((*(volatile unsigned int *)(GPIOA_START_ADDR + 0x10) >> 11) & 0x01)      // read
      {
         temp |= 1<< i;
      }
      if(i != 7)
      ds1302_clock();
   }
   *data = temp;
}
unsigned char ds1302_read(uint8_t addr)
{
   unsigned char data8bits=0;
   //1. CE high
      *(unsigned int *) (GPIOA_START_ADDR + 0x14) |= 1 << 10;      // //CE_DS1302 set Write
   //2. addr전송
   ds1302_tx(addr+0x01);
   //3. data읽어들인다.
   ds1302_rx(&data8bits);
   //4. CE low
    *(unsigned int *) (GPIOA_START_ADDR + 0x14) &= ~(1 << 10);   // CE_DS1302 Reset Write
   // 5. bcd to DEC
   return bcd2dec(data8bits);
}

void ds1302_read_time(void)
{
   ds_time.seconds = ds1302_read(ADDR_SECONDS);
   ds_time.minutes = ds1302_read(ADDR_MINUTES);
   ds_time.hours= ds1302_read(ADDR_HOURS);
}

void ds1302_read_date(void)
{
   ds_time.date = ds1302_read(ADDR_DATE);
   ds_time.month = ds1302_read(ADDR_MONTH);
   ds_time.dayofweek= ds1302_read(ADDR_DAYOFWEEK);
   ds_time.year = ds1302_read(ADDR_YEAR);
}


void enter_input(t_list* list)
{
   int access=0;
   static int one_time=0; //한번만 실행되도록 해주는 플래그
   unsigned char result[6];

   char lcd_buff[100]; //입,퇴실 시간을 출력하기 위해 필요한 buff


   if(key_value=='=')
   {
      queue[4]='\0';
      sprintf(result,"%s",queue);

      result_num = atoi(result);
      t_st *p = list->head;
      while (p != NULL)
      {
         if (p->num == result_num)
         {

            queue_init();
            lcd_init();
            lcd_set_cur(0,0);
            lcd_set_cur(1,0);
            printf("=....\n");

            if(p->enter.hour == NULL)
            {
               p->enter.hour=ds_time.hours;
               p->enter.min=ds_time.minutes;
               p->enter.sec=ds_time.seconds;
               lcd_init();
               lcd_set_cur(0,0);
               lcd_send_string("enter time");
               lcd_set_cur(1,0);
               sprintf(lcd_buff,"%d:%d:%d",p->enter.hour,p->enter.min,p->enter.sec);
               lcd_send_string(lcd_buff);
               servo_motor_main();
               dotmatrix_main_test();
               input_mode=0;
               p->enter_flag=1;
               flash_erase();
               flash_write_time((uint32_t *) &ds_time, sizeof(ds_time));
               flash_write((uint32_t *) &st, sizeof(st));

               return;
            }

            else
            {
               p->exit.hour=ds_time.hours;
               p->exit.min=ds_time.minutes;
               p->exit.sec=ds_time.seconds;
               lcd_init();
               lcd_set_cur(0,0);
               lcd_send_string("exit time");
               lcd_set_cur(1,0);
               sprintf(lcd_buff,"%d:%d:%d",p->exit.hour,p->exit.min,p->exit.sec);
               lcd_send_string(lcd_buff);
               servo_motor_main();
               dotmatrix_main_test();
               input_mode=0;
               p->enter_flag=2;
               flash_erase();
               flash_write_time((uint32_t *) &ds_time, sizeof(ds_time));
               flash_write((uint32_t *) &st, sizeof(st));

               return;
            }
         }
         p = p->next; //헤드 주소와 비교해서 일치하지 않았을때 p를 다음 node로 바꾸어서 p가 NULL이 될때까지 반복해서 비교
      }
      printf("no data\n");
      input_mode=0;
   }
}


void time_display(void)
{
   char lcd_buff[100];
   if(TIM11_1ms_ds1302 >= 1000)
   {
      TIM11_1ms_ds1302=0;

      printf("*** %4d-%2d-%2d %2d:%2d:%2d\n",
            ds_time.year+2000,
            ds_time.month,
            ds_time.date,
            ds_time.hours,
            ds_time.minutes,
            ds_time.seconds);
      sprintf(lcd_buff,"%d-%d-%d        ",ds_time.year+2000,ds_time.month,ds_time.date);
      lcd_set_cur(0,0);
      lcd_send_string(lcd_buff);
      sprintf(lcd_buff,"%d:%d:%d        ",ds_time.hours,ds_time.minutes,ds_time.seconds);
      lcd_set_cur(1,0);
      lcd_send_string(lcd_buff);
   }
}


void show(t_list *list)
{
   int id_access=0;
   char lcd_buff[20];
   unsigned char result[6];
   static int show_id =0;


   t_st *p = list->head;
   if(show_id==0)
   {
      if(queue[4]=='=')
      {
         queue[4]='\0';
         sprintf(result,"%s",queue);
         result_num=atoi(result);
         while (p != NULL)
         {
            printf("find\n");
            if (p->num==result_num)
            {
               printf("success\n");
               lcd_init();
               queue_init();
               show_id=1;
               id_access=1;
               break;
            }
            p = p->next; //헤드 주소와 비교해서 일치하지 않았을때 p를 다음 node로 바꾸어서 p가 NULL이 될때까지 반복해서 비교
         }
         if(id_access==0){
            lcd_init();
            lcd_set_cur(0,0);
            lcd_send_string("wrong id");
            HAL_Delay(1000);
            return;
         }
      }
   }

   if(show_id ==1)
   {
      if(p->enter_flag==1)
      {
         lcd_init();
         lcd_set_cur(0,0);
         lcd_send_string("enter:");
         sprintf(lcd_buff,"%d:%d:%d",p->enter.hour,p->enter.min,p->enter.sec);
         lcd_send_string(lcd_buff);
         id_access=0;
      }

      if(p->enter_flag==2)
      {
         lcd_init();
         lcd_set_cur(0,0);
         lcd_send_string("enter:");
         sprintf(lcd_buff,"%d:%d:%d",p->enter.hour,p->enter.min,p->enter.sec);
         lcd_send_string(lcd_buff);
         lcd_set_cur(1,0);
         lcd_send_string("exit : ");
         sprintf(lcd_buff,"%d:%d:%d",p->exit.hour,p->exit.min,p->exit.sec);
         lcd_send_string(lcd_buff);
         id_access=0;
      }

      if(p->enter_flag==0)
      {
         lcd_init();
         lcd_set_cur(0,0);
         lcd_send_string("no data");
         id_access=0;
      }
      HAL_Delay(2000);
   }
   select_=0;
   start=0;
   show_id=0;
   input_mode=0;
   return;
}


void time_reset(void)
{
   char lcd_timereset_buff[100];

      if(hour_reset_button==1)
      {
         if(ds_time.hours>=24)
         {
            ds_time.hours=00;
         }
         else
         {
         ds_time.hours++;
         }
         hour_reset_button=0;
printf("hour_reset_button_pressed\n");
      }
      if(min_reset_button==1)
      {
         if(ds_time.minutes>=60)
            ds_time.minutes=00;
         ds_time.minutes++;
printf("minute_reset_button_pressed\n");
         min_reset_button=0;
      }

      if(sec_reset_button==1)
      {
         if(ds_time.seconds>=60)
            ds_time.seconds=00;
         ds_time.seconds++;
         sec_reset_button=0;
printf("sec_reset_button_pressed\n");
      }


      lcd_set_cur(0,0);
      lcd_send_string("BUTTON3 TO SET      ");
      sprintf(lcd_timereset_buff,"%d:%d:%d        ",ds_time.hours,ds_time.minutes,ds_time.seconds);
      lcd_set_cur(1,0);
      lcd_send_string(lcd_timereset_buff);

      if(get_button(GPIOC,GPIO_PIN_9,BUTTON3)==BUTTON_PRESS)
      {
         ds1302_gpio_init();
         ds1302_init();

         time_reset_run=0;
         ds1302_run=0;
         input_mode=0;

      }
}

