/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LedePCI_Pin GPIO_PIN_13
#define LedePCI_GPIO_Port GPIOC
#define Caixa_NivelAlto_Pin GPIO_PIN_0
#define Caixa_NivelAlto_GPIO_Port GPIOA
#define Caixa_NivelAlto_EXTI_IRQn EXTI0_IRQn
#define Caixa_NivelBaixo_Pin GPIO_PIN_1
#define Caixa_NivelBaixo_GPIO_Port GPIOA
#define Poco_NivelAlto_Pin GPIO_PIN_2
#define Poco_NivelAlto_GPIO_Port GPIOA
#define Poco_NivelBaixo_Pin GPIO_PIN_3
#define Poco_NivelBaixo_GPIO_Port GPIOA
#define Poco_NivelBaixo_EXTI_IRQn EXTI3_IRQn
#define ligarMotor1min_Pin GPIO_PIN_5
#define ligarMotor1min_GPIO_Port GPIOA
#define ligarMotor1min_EXTI_IRQn EXTI9_5_IRQn
#define ligarMotor5min_Pin GPIO_PIN_6
#define ligarMotor5min_GPIO_Port GPIOA
#define ligarMotor5min_EXTI_IRQn EXTI9_5_IRQn
#define ligarMotor10min_Pin GPIO_PIN_7
#define ligarMotor10min_GPIO_Port GPIOA
#define ligarMotor10min_EXTI_IRQn EXTI9_5_IRQn
#define Motor_Pin GPIO_PIN_6
#define Motor_GPIO_Port GPIOB
#define LedAzul_Pin GPIO_PIN_7
#define LedAzul_GPIO_Port GPIOB
#define LedVerde_Pin GPIO_PIN_8
#define LedVerde_GPIO_Port GPIOB
#define LedAmarelo_Pin GPIO_PIN_9
#define LedAmarelo_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
