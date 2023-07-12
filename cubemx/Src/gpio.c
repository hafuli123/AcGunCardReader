/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "dev.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PWR_EN */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = PWR_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PWR_EN_GPIO, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_DONE */
  GPIO_InitStruct.Pin = BAT_DONE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_DONE_GPIO, &GPIO_InitStruct);
  /*Configure GPIO pin : BAT_CHRG */
  GPIO_InitStruct.Pin = BAT_CHRG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_CHRG_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = KEY_ON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_ON_GPIO, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_2 */
  GPIO_InitStruct.Pin = KEY_2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_2_GPIO, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
device led={
        "led",
        led_open,
        led_read,
        led_write,
        NULL
};

uint8_t led_open(const char * pathname , int flags)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pin : LED */
    GPIO_InitStruct.Pin = LED_R_PIN|LED_G_PIN|LED_Y_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED_X_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_X_GPIO, LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_X_GPIO, LED_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_X_GPIO, LED_Y_PIN, GPIO_PIN_SET);
    return 0;
}
__IO    uint8_t v;
uint8_t led_read(int fd,void*buf,int count)
{
    switch(count){
    case LED_G:{
        if(HAL_GPIO_ReadPin(LED_X_GPIO, LED_G_PIN)==GPIO_PIN_RESET){v=0;}
        else{v=1;}
        return v;
    }
    case LED_R:{
        if(HAL_GPIO_ReadPin(LED_X_GPIO, LED_R_PIN)==GPIO_PIN_RESET){v=0;}
        else{v=1;}
        return v;
    }
    case LED_Y:{
        if(HAL_GPIO_ReadPin(LED_X_GPIO, LED_Y_PIN)==GPIO_PIN_RESET){v=0;}
        else{v=1;}
        return v;
    }
    }
    return 0;
}
uint8_t led_write(int fd,void*buf,int count)
{
    uint8_t *val;
    val=(uint8_t*)buf;
    switch(count){
    case LED_G:{
        if(*val==LED_ON){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_G_PIN, GPIO_PIN_RESET);
            return 0;
        }
        else if(*val==LED_OFF){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_G_PIN, GPIO_PIN_SET);
            return 0;
        }
    }
    case LED_R:{
        if(*val==LED_ON){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_R_PIN, GPIO_PIN_RESET);
            return 0;
        }
        else if(*val==LED_OFF){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_R_PIN, GPIO_PIN_SET);
            return 0;
        }
    }
    case LED_Y:{
        if(*val==LED_ON){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_Y_PIN, GPIO_PIN_RESET);
            return 0;
        }
        else if(*val==LED_OFF){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_Y_PIN, GPIO_PIN_SET);
            return 0;
        }
    }
    }


    return 0;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == KEY_ON_PIN){
//        rt_thread_mdelay(1);
//        if(HAL_GPIO_ReadPin(KEY_ON_GPIO,KEY_ON_PIN)==KEYON_DOWN_LEVEL){
//
//
//        }
//    }
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
//    return;
//}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
