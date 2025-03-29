#include "lcd.h"

#define GPIOB_START_ADDR 0x40020400
#define GPIOC_START_ADDR 0x40020800


void pulse_en() {

	*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= 1 << 8;		// //E Write set
    delay_us(20);
	*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 8);	// E Write Reset
    delay_us(20);
}

//----- LCD 디스플레이로 4bit 데이터 전송
void lcd_send_4bit(char data) {
	((data >> 3) & 0x01) ? (*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= (1 << 7)) : (*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 7));	//LCD7
	((data >> 2) & 0x01) ? (*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= (1 << 6)) : (*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 6));	//LCD6
	((data >> 1) & 0x01) ? (*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= (1 << 5)) : (*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 5));	//LCD5
	(data& 0x01) ? (*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= (1 << 4)) : (*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 4));				//LCD4
    pulse_en();
}

//----- LCD 디스플레이로 8bit 데이터 전송
void lcd_send_8bit(char data) {
	lcd_send_4bit((data >> 4) & 0x0F); // 상위 4비트 전송
	lcd_send_4bit(data & 0x0F);        // 하위 4비트 전송
}

//----- LCD 디스플레이 커서 설정
void lcd_set_cur(int row, int col) {
	*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 10);	// R_S Write Reset
    switch (row) {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_8bit(col); // DDRAM AD Set 명령
}

//----- LCD 디스플레이 문자 비우기
void lcd_clear() {
	*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 10);	// R_S Write Reset
	lcd_send_8bit(0b00000001); // Screen Clear 명령 -> 디스플레이 내용 지움
	HAL_Delay(2);              // 명령 실행시간인 1.64ms 이상 대기
}

//----- LCD 디스플레이로 문자열 전송
void lcd_send_string(char *str) {
	*(unsigned int *) (GPIOB_START_ADDR + 0x14) |= 1 << 10;		// /R_SE Write set
	// 보낼 string 주소값(포인터)을 받아 문자 하나씩 lcd_send_8bit 함수로 보내며 포인터 1씩 올림(=다음 문자 선택)
	while (*str) {
		lcd_send_8bit(*str++);
	}
}

//----- LCD 디스플레이 초기화 (데이터시트 INITIALIZATION SEQUENCE 참고)
void lcd_init(void) {
	*(unsigned int *) (GPIOB_START_ADDR + 0x14) &= ~(1 << 10);	// R_S Write Reset

    // 4 bit 모드로 설정 (데이터시트를 따름)
    HAL_Delay(50);         // 부팅시간인 15ms 이상 대기

    lcd_send_4bit(0b0011); // 설정 반복 1
    HAL_Delay(5);          // 명령 실행시간인 4.1ms 이상 대기
    lcd_send_4bit(0b0011); // 설정 반복 2
    HAL_Delay(1);          // 명령 실행시간인 0.1ms 이상 대기 (이하 동일)
    lcd_send_4bit(0b0011); // 설정 반복 3
    HAL_Delay(1);
    lcd_send_4bit(0b0010); // 4비트 모드로 설정
    HAL_Delay(1);

    // 디스플레이 Init (데이터시트를 따름)
    lcd_send_8bit(0b00101000); // Function set 명령 -> DL=0, N=1, F=0 -> 4비트, 2줄, 5x7 도트
    HAL_Delay(1);              // 명령 실행시간인 0.04ms 이상 대기 (이하 동일)
    lcd_send_8bit(0b00001000); // Display Switch 명령 -> D=0, C=0, B=0  -> 디스플레이 OFF, 커서 OFF, 블링크 OFF
    HAL_Delay(1);
    lcd_send_8bit(0b00000001); // Screen Clear 명령 -> 디스플레이 내용 지움
    HAL_Delay(2);
    lcd_send_8bit(0b00000110); // Input Set 명령 -> I/D = 1, S = 0 -> 커서 방향 더하기, 시프트 X
    HAL_Delay(1);

    // Init 끝. 사용을 위해 디스플레이 켜줌.
    lcd_send_8bit(0b00001100); // Display Switch 명령 -> D=0, C=0, B=0  -> 디스플레이 ON, 커서 OFF, 블링크 OFF
    HAL_Delay(1);
}
/* USER CODE END 0 */
