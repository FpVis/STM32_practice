/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ROW1_KEYPAD_Pin GPIO_PIN_0
#define ROW1_KEYPAD_GPIO_Port GPIOC
#define ROW2_KEYPAD_Pin GPIO_PIN_1
#define ROW2_KEYPAD_GPIO_Port GPIOC
#define ROW3_KEYPAD_Pin GPIO_PIN_2
#define ROW3_KEYPAD_GPIO_Port GPIOC
#define ROW4_KEYPAD_Pin GPIO_PIN_3
#define ROW4_KEYPAD_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define SERVO_MOTOR_Pin GPIO_PIN_6
#define SERVO_MOTOR_GPIO_Port GPIOA
#define SERVO_CONTROL_Pin GPIO_PIN_5
#define SERVO_CONTROL_GPIO_Port GPIOC
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
#define LED_TIM4_CH1_Pin GPIO_PIN_6
#define LED_TIM4_CH1_GPIO_Port GPIOB

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
	uint8_t am_pm;
	uint8_t hour_mode;
	uint8_t dummy[3];

}t_ds1302;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
