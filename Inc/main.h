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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_INTN_Pin GPIO_PIN_3
#define SPI4_INTN_GPIO_Port GPIOE
#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
#define INTEL_REST_Pin GPIO_PIN_13
#define INTEL_REST_GPIO_Port GPIOC
#define OUT_1_Pin GPIO_PIN_0
#define OUT_1_GPIO_Port GPIOC
#define OUT_2_Pin GPIO_PIN_1
#define OUT_2_GPIO_Port GPIOC
#define OUT_3_Pin GPIO_PIN_2
#define OUT_3_GPIO_Port GPIOC
#define OUT_4_Pin GPIO_PIN_3
#define OUT_4_GPIO_Port GPIOC
#define OUT_5_Pin GPIO_PIN_0
#define OUT_5_GPIO_Port GPIOA
#define OUT_6_Pin GPIO_PIN_1
#define OUT_6_GPIO_Port GPIOA
#define USART2_RD_Pin GPIO_PIN_4
#define USART2_RD_GPIO_Port GPIOA
#define OUT_10_Pin GPIO_PIN_6
#define OUT_10_GPIO_Port GPIOA
#define OUT_9_Pin GPIO_PIN_4
#define OUT_9_GPIO_Port GPIOC
#define OUT_8_Pin GPIO_PIN_5
#define OUT_8_GPIO_Port GPIOC
#define OUT_7_Pin GPIO_PIN_0
#define OUT_7_GPIO_Port GPIOB
#define UART7_RD_Pin GPIO_PIN_9
#define UART7_RD_GPIO_Port GPIOE
#define IN_1_Pin GPIO_PIN_10
#define IN_1_GPIO_Port GPIOE
#define IN_2_Pin GPIO_PIN_11
#define IN_2_GPIO_Port GPIOE
#define SD_Select_Pin GPIO_PIN_14
#define SD_Select_GPIO_Port GPIOE
#define SLEEP_Pin GPIO_PIN_15
#define SLEEP_GPIO_Port GPIOE
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define USART3_RD_Pin GPIO_PIN_10
#define USART3_RD_GPIO_Port GPIOD
#define IN_3_Pin GPIO_PIN_11
#define IN_3_GPIO_Port GPIOD
#define IN_4_Pin GPIO_PIN_14
#define IN_4_GPIO_Port GPIOD
#define IN_4_EXTI_IRQn EXTI15_10_IRQn
#define USART6_RD_Pin GPIO_PIN_15
#define USART6_RD_GPIO_Port GPIOD
#define IN_5_Pin GPIO_PIN_8
#define IN_5_GPIO_Port GPIOA
#define IN_5_EXTI_IRQn EXTI9_5_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IN_6_Pin GPIO_PIN_15
#define IN_6_GPIO_Port GPIOA
#define IN_6_EXTI_IRQn EXTI15_10_IRQn
#define IN_7_Pin GPIO_PIN_3
#define IN_7_GPIO_Port GPIOD
#define IN_7_EXTI_IRQn EXTI3_IRQn
#define IN_8_Pin GPIO_PIN_4
#define IN_8_GPIO_Port GPIOD
#define IN_9_Pin GPIO_PIN_5
#define IN_9_GPIO_Port GPIOD
#define IN_10_Pin GPIO_PIN_6
#define IN_10_GPIO_Port GPIOD
#define IN_11_Pin GPIO_PIN_7
#define IN_11_GPIO_Port GPIOD
#define IN_12_Pin GPIO_PIN_3
#define IN_12_GPIO_Port GPIOB
#define IN_13_Pin GPIO_PIN_4
#define IN_13_GPIO_Port GPIOB
#define IN_14_Pin GPIO_PIN_5
#define IN_14_GPIO_Port GPIOB
#define IN_15_Pin GPIO_PIN_6
#define IN_15_GPIO_Port GPIOB
#define IN_16_Pin GPIO_PIN_7
#define IN_16_GPIO_Port GPIOB
#define IN_17_Pin GPIO_PIN_8
#define IN_17_GPIO_Port GPIOB
#define IN_18_Pin GPIO_PIN_9
#define IN_18_GPIO_Port GPIOB
#define IN_19_Pin GPIO_PIN_0
#define IN_19_GPIO_Port GPIOE
#define IN_20_Pin GPIO_PIN_1
#define IN_20_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
