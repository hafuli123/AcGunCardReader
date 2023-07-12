/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "dev.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define KEY_ON_GPIO     GPIOC
#define KEY_ON_PIN      GPIO_PIN_0
#define KEY_ON_DOWN_LEVEL GPIO_PIN_RESET

#define KEY_2_GPIO     GPIOC
#define KEY_2_PIN      GPIO_PIN_1
#define KEY_2_DOWN_LEVEL GPIO_PIN_RESET
#define KEY_READ(pin)   HAL_GPIO_ReadPin(GPIOC,pin)

#define BAT_DONE_GPIO     GPIOC
#define BAT_DONE_PIN      GPIO_PIN_8
#define BAT_DONE_Y        (HAL_GPIO_ReadPin(GPIOC,BAT_DONE_PIN)==GPIO_PIN_RESET)
#define BAT_DONE_N        (HAL_GPIO_ReadPin(GPIOC,BAT_DONE_PIN)==GPIO_PIN_SET)

#define BAT_CHRG_GPIO     GPIOC
#define BAT_CHRG_PIN      GPIO_PIN_9
#define BAT_CHRG_Y       (HAL_GPIO_ReadPin(GPIOC,BAT_CHRG_PIN)==GPIO_PIN_SET)
#define BAT_CHRG_N       (HAL_GPIO_ReadPin(GPIOC,BAT_CHRG_PIN)==GPIO_PIN_RESET)

#define PWR_EN_GPIO     GPIOB
#define PWR_EN_PIN      GPIO_PIN_3
#define SYSTEM_POWER_ON()   HAL_GPIO_WritePin(PWR_EN_GPIO, PWR_EN_PIN, GPIO_PIN_SET)
#define SYSTEM_POWER_OFF()   HAL_GPIO_WritePin(PWR_EN_GPIO, PWR_EN_PIN, GPIO_PIN_RESET)

#define LED_X_GPIO     GPIOB
#define LED_R_PIN      GPIO_PIN_5
#define LED_G_PIN      GPIO_PIN_6
#define LED_Y_PIN      GPIO_PIN_7
#define LED_R   0
#define LED_G   1
#define LED_Y   2
#define LED_R_SK   3
#define LED_G_SK   4
#define LED_Y_SK    5
#define LED_ON  0
#define LED_OFF 1
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

extern device led;

uint8_t led_open(const char * pathname , int flags);
uint8_t led_read(int fd,void*buf,int count);
uint8_t led_write(int fd,void*buf,int count);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
