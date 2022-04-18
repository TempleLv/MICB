/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define RLY_1_Pin GPIO_PIN_1
#define RLY_1_GPIO_Port GPIOC
#define RLY_2_Pin GPIO_PIN_2
#define RLY_2_GPIO_Port GPIOC
#define RLY_3_Pin GPIO_PIN_3
#define RLY_3_GPIO_Port GPIOC
#define Ipwm_2_Pin GPIO_PIN_0
#define Ipwm_2_GPIO_Port GPIOA
#define Ipwm_1_Pin GPIO_PIN_1
#define Ipwm_1_GPIO_Port GPIOA
#define AV_3_Pin GPIO_PIN_4
#define AV_3_GPIO_Port GPIOA
#define AV_4_Pin GPIO_PIN_5
#define AV_4_GPIO_Port GPIOA
#define AV_1_Pin GPIO_PIN_6
#define AV_1_GPIO_Port GPIOA
#define AV_2_Pin GPIO_PIN_7
#define AV_2_GPIO_Port GPIOA
#define Vfb_2_Pin GPIO_PIN_4
#define Vfb_2_GPIO_Port GPIOC
#define Vfb_1_Pin GPIO_PIN_5
#define Vfb_1_GPIO_Port GPIOC
#define Vfb_4_Pin GPIO_PIN_0
#define Vfb_4_GPIO_Port GPIOB
#define Vfb_3_Pin GPIO_PIN_1
#define Vfb_3_GPIO_Port GPIOB
#define RSTn_Pin GPIO_PIN_2
#define RSTn_GPIO_Port GPIOB
#define Mpwm_1_Pin GPIO_PIN_10
#define Mpwm_1_GPIO_Port GPIOB
#define Mpwm_2_Pin GPIO_PIN_11
#define Mpwm_2_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Vpwm_1_Pin GPIO_PIN_6
#define Vpwm_1_GPIO_Port GPIOC
#define Vpwm_2_Pin GPIO_PIN_7
#define Vpwm_2_GPIO_Port GPIOC
#define Vpwm_3_Pin GPIO_PIN_8
#define Vpwm_3_GPIO_Port GPIOC
#define Vpwm_4_Pin GPIO_PIN_9
#define Vpwm_4_GPIO_Port GPIOC
#define INTn_Pin GPIO_PIN_8
#define INTn_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define debug_TX_Pin GPIO_PIN_10
#define debug_TX_GPIO_Port GPIOC
#define debug_RX_Pin GPIO_PIN_11
#define debug_RX_GPIO_Port GPIOC
#define RD_Pin GPIO_PIN_6
#define RD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
