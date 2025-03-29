#include "main.h"

// 독립적인 변수 선언
volatile uint8_t is_first_capture_left = 0;   // 0: 상승에지, 1: 하강에지
volatile uint8_t is_first_capture_center = 0;
volatile uint8_t is_first_capture_right = 0;

volatile int distance_left = 0;
volatile int distance_center = 0;
volatile int distance_right = 0;

volatile int ic_cpt_finish_flag_left = 0;    // 센서별 측정 완료 플래그
volatile int ic_cpt_finish_flag_center = 0;
volatile int ic_cpt_finish_flag_right = 0;

int dis_left, dis_center, dis_right;

extern volatile int TIM11_1ms_ultrasonic;
// 타이머 입력 캡처 콜백 함수 수정
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // 왼쪽 센서 채널 처리
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // TIM_CHANNEL_1
    {
        if (is_first_capture_left == 0)  // 상승 에지
        {
            __HAL_TIM_SET_COUNTER(htim, 0); // 타이머 초기화
            is_first_capture_left = 1;
        }
        else if (is_first_capture_left == 1)  // 하강 에지
        {
            is_first_capture_left = 0;
            distance_left = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // 값 읽기
            ic_cpt_finish_flag_left = 1;  // 측정 완료 플래그 설정
        }
    }

    // 중앙 센서 채널 처리
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // TIM_CHANNEL_2
    {
        if (is_first_capture_center == 0)  // 상승 에지
        {
            __HAL_TIM_SET_COUNTER(htim, 0); // 타이머 초기화
            is_first_capture_center = 1;
        }
        else if (is_first_capture_center == 1)  // 하강 에지
        {
            is_first_capture_center = 0;
            distance_center = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // 값 읽기
            ic_cpt_finish_flag_center = 1;  // 측정 완료 플래그 설정
        }
    }

    // 오른쪽 센서 채널 처리
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // TIM_CHANNEL_4
    {
        if (is_first_capture_right == 0)  // 상승 에지
        {
            __HAL_TIM_SET_COUNTER(htim, 0); // 타이머 초기화
            is_first_capture_right = 1;
        }
        else if (is_first_capture_right == 1)  // 하강 에지
        {
            is_first_capture_right = 0;
            distance_right = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // 값 읽기
            ic_cpt_finish_flag_right = 1;  // 측정 완료 플래그 설정
        }
    }
}

// 거리 계산 함수 수정
void ultrasonic_processing(void)
{

    char lcd_buff[20];

    if (TIM11_1ms_ultrasonic >= 60)  // 1초마다 초음파 측정
    {
        TIM11_1ms_ultrasonic = 0;
        make_trigger();

        // 왼쪽 센서 처리
        if (ic_cpt_finish_flag_left)
        {
            ic_cpt_finish_flag_left = 0;
            dis_left = distance_left * 0.034 / 2;  // 거리 계산
        }

        // 중앙 센서 처리
        if (ic_cpt_finish_flag_center)
        {
            ic_cpt_finish_flag_center = 0;
            dis_center = distance_center * 0.034 / 2;  // 거리 계산
        }

        // 오른쪽 센서 처리
        if (ic_cpt_finish_flag_right)
        {
            ic_cpt_finish_flag_right = 0;
            dis_right = distance_right * 0.034 / 2;  // 거리 계산
        }

        // 결과 LCD 출력
        sprintf(lcd_buff, "%dcm %dcm %dcm     ", dis_left, dis_center, dis_right);
        move_cursor(1, 0);
        lcd_string(lcd_buff);
    }
}
void make_trigger(void)
{
    // 왼쪽 초음파 센서 트리거
    HAL_GPIO_WritePin(LEFT_TRIG_GPIO_Port, LEFT_TRIG_Pin, 0); // 트리거 핀 LOW
    delay_us(2);                                             // 2μs 대기
    HAL_GPIO_WritePin(LEFT_TRIG_GPIO_Port, LEFT_TRIG_Pin, 1); // 트리거 핀 HIGH
    delay_us(10);                                            // 10μs 펄스 유지
    HAL_GPIO_WritePin(LEFT_TRIG_GPIO_Port, LEFT_TRIG_Pin, 0); // 트리거 핀 LOW

    // 중앙 초음파 센서 트리거
    HAL_GPIO_WritePin(CENTER_TRIG_GPIO_Port, CENTER_TRIG_Pin, 0);
    delay_us(2);
    HAL_GPIO_WritePin(CENTER_TRIG_GPIO_Port, CENTER_TRIG_Pin, 1);
    delay_us(10);
    HAL_GPIO_WritePin(CENTER_TRIG_GPIO_Port, CENTER_TRIG_Pin, 0);

    // 오른쪽 초음파 센서 트리거
    HAL_GPIO_WritePin(RIGHT_TRIG_GPIO_Port, RIGHT_TRIG_Pin, 0);
    delay_us(2);
    HAL_GPIO_WritePin(RIGHT_TRIG_GPIO_Port, RIGHT_TRIG_Pin, 1);
    delay_us(10);
    HAL_GPIO_WritePin(RIGHT_TRIG_GPIO_Port, RIGHT_TRIG_Pin, 0);
}

