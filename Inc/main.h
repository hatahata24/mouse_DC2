/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define min(A,B) ((A)>(B))?(B):(A)
#define max(A,B) ((A)>(B))?(A):(B)

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
#define FLED2_RED_Pin GPIO_PIN_13
#define FLED2_RED_GPIO_Port GPIOC
#define FLED2_GREEN_Pin GPIO_PIN_14
#define FLED2_GREEN_GPIO_Port GPIOC
#define FLED2_BLUE_Pin GPIO_PIN_15
#define FLED2_BLUE_GPIO_Port GPIOC
#define IR_L_Pin GPIO_PIN_0
#define IR_L_GPIO_Port GPIOH
#define IR_FL_Pin GPIO_PIN_1
#define IR_FL_GPIO_Port GPIOH
#define VAL_FR_Pin GPIO_PIN_0
#define VAL_FR_GPIO_Port GPIOC
#define VAL_R_Pin GPIO_PIN_1
#define VAL_R_GPIO_Port GPIOC
#define VAL_L_Pin GPIO_PIN_2
#define VAL_L_GPIO_Port GPIOC
#define VAL_FL_Pin GPIO_PIN_3
#define VAL_FL_GPIO_Port GPIOC
#define EC_L_A_Pin GPIO_PIN_0
#define EC_L_A_GPIO_Port GPIOA
#define EC_L_B_Pin GPIO_PIN_1
#define EC_L_B_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_2
#define LED6_GPIO_Port GPIOA
#define VAT_ALERT_Pin GPIO_PIN_3
#define VAT_ALERT_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_4
#define LED7_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define MOTOR_L_PWM_Pin GPIO_PIN_0
#define MOTOR_L_PWM_GPIO_Port GPIOB
#define MOTOR_L_CCW_Pin GPIO_PIN_1
#define MOTOR_L_CCW_GPIO_Port GPIOB
#define MOTOR_L_CW_Pin GPIO_PIN_10
#define MOTOR_L_CW_GPIO_Port GPIOB
#define MOTOR_L_R_STBY_Pin GPIO_PIN_11
#define MOTOR_L_R_STBY_GPIO_Port GPIOB
#define MOTOR_R_CW_Pin GPIO_PIN_12
#define MOTOR_R_CW_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_13
#define LED5_GPIO_Port GPIOB
#define MOTOR_R_CCW_Pin GPIO_PIN_14
#define MOTOR_R_CCW_GPIO_Port GPIOB
#define EC_R_A_Pin GPIO_PIN_6
#define EC_R_A_GPIO_Port GPIOC
#define EC_R_B_Pin GPIO_PIN_7
#define EC_R_B_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOC
#define VOL_CHECK_Pin GPIO_PIN_9
#define VOL_CHECK_GPIO_Port GPIOC
#define PUSH_IN_Pin GPIO_PIN_8
#define PUSH_IN_GPIO_Port GPIOA
#define IR_FR_Pin GPIO_PIN_11
#define IR_FR_GPIO_Port GPIOA
#define IR_R_Pin GPIO_PIN_12
#define IR_R_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define FLED1_BLUE_Pin GPIO_PIN_5
#define FLED1_BLUE_GPIO_Port GPIOB
#define FLED1_RED_Pin GPIO_PIN_6
#define FLED1_RED_GPIO_Port GPIOB
#define FLED1_GREEN_Pin GPIO_PIN_7
#define FLED1_GREEN_GPIO_Port GPIOB
#define BUZZER_PWM_Pin GPIO_PIN_8
#define BUZZER_PWM_GPIO_Port GPIOB
#define VACUUM_PWM_Pin GPIO_PIN_9
#define VACUUM_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
