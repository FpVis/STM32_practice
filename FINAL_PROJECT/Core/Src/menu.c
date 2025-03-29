#include "main.h"
#include "lcd.h"
#include "circularQueue.h"

void show_menu(void) ;
void make(void);
void input(void);
void del(void);
void check1(void);


extern void button_check(void);
extern unsigned char queue[QUEUE_MAX];

int menu_flg = 0;

void show_menu(void)
{

    lcd_init();
    lcd_set_cur(0, 0);
    lcd_send_string("1.input 2.check   ");
    lcd_set_cur(1, 0);
    lcd_send_string("3.time change   ");
    menu_flg = 1;
}
