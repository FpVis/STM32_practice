/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ROW1_KEYPAD_Pin GPIO_PIN_0
#define ROW1_KEYPAD_GPIO_Port GPIOC
#define ROW2_KEYPAD_Pin GPIO_PIN_1
#define ROW2_KEYPAD_GPIO_Port GPIOC
#define ROW3_KEYPAD_Pin GPIO_PIN_2
#define ROW3_KEYPAD_GPIO_Port GPIOC
#define ROW4_KEYPAD_Pin GPIO_PIN_3
#define ROW4_KEYPAD_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOA
#define SERVO_MOTOR_Pin GPIO_PIN_6
#define SERVO_MOTOR_GPIO_Port GPIOA
#define BUTTON_0_Pin GPIO_PIN_5
#define BUTTON_0_GPIO_Port GPIOC
#define LCD0_Pin GPIO_PIN_0
#define LCD0_GPIO_Port GPIOB
#define LCD1_Pin GPIO_PIN_1
#define LCD1_GPIO_Port GPIOB
#define LCD2_Pin GPIO_PIN_2
#define LCD2_GPIO_Port GPIOB
#define R_S_Pin GPIO_PIN_10
#define R_S_GPIO_Port GPIOB
#define COL1_Pin GPIO_PIN_12
#define COL1_GPIO_Port GPIOB
#define COL2_Pin GPIO_PIN_13
#define COL2_GPIO_Port GPIOB
#define COL3_Pin GPIO_PIN_14
#define COL3_GPIO_Port GPIOB
#define COL4_Pin GPIO_PIN_15
#define COL4_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_6
#define BUTTON_1_GPIO_Port GPIOC
#define BUTTON_2_Pin GPIO_PIN_8
#define BUTTON_2_GPIO_Port GPIOC
#define BUTTON_3_Pin GPIO_PIN_9
#define BUTTON_3_GPIO_Port GPIOC
#define CE_DS1302_Pin GPIO_PIN_10
#define CE_DS1302_GPIO_Port GPIOA
#define IO_DS1302_Pin GPIO_PIN_11
#define IO_DS1302_GPIO_Port GPIOA
#define CLK_DS1302_Pin GPIO_PIN_12
#define CLK_DS1302_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LATCH_Pin GPIO_PIN_15
#define LATCH_GPIO_Port GPIOA
#define LCD3_Pin GPIO_PIN_3
#define LCD3_GPIO_Port GPIOB
#define LCD4_Pin GPIO_PIN_4
#define LCD4_GPIO_Port GPIOB
#define LCD5_Pin GPIO_PIN_5
#define LCD5_GPIO_Port GPIOB
#define LCD6_Pin GPIO_PIN_6
#define LCD6_GPIO_Port GPIOB
#define LCD7_Pin GPIO_PIN_7
#define LCD7_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_8
#define E_GPIO_Port GPIOB
#define R_W_Pin GPIO_PIN_9
#define R_W_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef struct ds1302
{
	uint32_t magic;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t date;
	uint8_t month;
	uint8_t dayofweek;
	uint8_t year;
	uint8_t ampm;
	uint8_t hourmode; // 0 = 24mode, 1 = 12mode
	uint8_t dummy[3];  // --32bits

} t_ds1302;

typedef struct {
	struct st* head;      // Î¶¨Ïä§?ä∏?ùò Ï≤? Î≤àÏß∏ ?Ö∏?ìúÎ•? ???û•
	struct st* tail;      // Î¶¨Ïä§?ä∏?ùò ÎßàÏ?Îß? ?Ö∏?ìúÎ•? ???û•
	uint8_t num;  //?Ç¨?õê ?àò1 8
	uint8_t dummy[3];
} t_list;

typedef struct
{
	uint8_t hour;        // ?ãúÍ∞?
	uint8_t min;  // Î∂?
    uint8_t sec;   // Ï¥?
} time_data;


typedef struct st
{
	uint16_t  num;        // ?Ç¨?õêÎ≤àÌò∏  16
	uint16_t password;  // ÎπÑÎ?Î≤àÌò∏  16
    time_data enter;  // Ï∂úÍ∑º?ãúÍ∞?
    time_data exit;  // ?á¥Í∑ºÏãúÍ∞?
    uint16_t enter_flag;
	struct st* prev; // ?ù¥?†Ñ ?Ö∏?ìú ?è¨?ù∏?Ñ∞
	struct st* next; // ?ã§?ùå ?Ö∏?ìú ?è¨?ù∏?Ñ∞
}t_st;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
