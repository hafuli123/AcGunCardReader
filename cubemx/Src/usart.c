/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <rtthread.h>
#include "dev.h"
#include <string.h>
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef husartx;
/* USART1 init function */

__IO uint8_t aRxBuffer[RX_MAX_COUNT]={0};
__IO uint16_t RxCount=0;
__IO uint8_t Frame_flag=0;

uint8_t atmode0[]="AT+MODE=0\r\n";

device gc24={
        "gc24",
        gc24_open,
        gc24_read,
        gc24_write,
        gc24_ioctl
};

uint8_t gc24_open(const char * pathname , int flags)
{
    MX_USARTx_Init();
    __HAL_UART_ENABLE_IT(&husartx,UART_IT_RXNE);
    return 0;
}
uint8_t gc24_read(int fd,void*buf,int count)
{
    if(Frame_flag){
        uint8_t *p;
        p=buf;
        RxCount=0;
        Frame_flag=0;
        for(int i=0;i<RX_MAX_COUNT;i++){
            *p=aRxBuffer[i];
            p++;
        }
        memset((uint8_t*)aRxBuffer,0,RX_MAX_COUNT);
    }
    return 0;
}
uint8_t gc24_write(int fd,void*buf,int count)
{
    uint8_t *p;
    p=buf;
    HAL_UART_Transmit(&husartx,p,count,3);
    return 0;
}
uint8_t gc24_ioctl(int fd, int val,int mod)
{

    return 0;
}



/* USER CODE BEGIN 1 */
//GC2400模块串口1配置
static void MX_NVIC_USARTx_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void MX_USARTx_Init(void)
{
  /* 串口外设时钟使能 */
  USART_RCC_CLK_ENABLE();

  husartx.Instance = USARTx;
  husartx.Init.BaudRate = USARTx_BAUDRATE;
  husartx.Init.WordLength = UART_WORDLENGTH_8B;
  husartx.Init.StopBits = UART_STOPBITS_1;
  husartx.Init.Parity = UART_PARITY_NONE;
  husartx.Init.Mode = UART_MODE_TX_RX;
  husartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx);

  /* 配置串口中断并使能，需要放在HAL_UART_Init函数后执行修改才有效 */
  MX_NVIC_USARTx_Init();
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USARTx)
  {
      /* 使能串口功能引脚GPIO时钟 */
      USARTx_GPIO_ClK_ENABLE();

      /* 串口外设功能GPIO配置 */
      GPIO_InitStruct.Pin = USARTx_Tx_GPIO_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = USARTx_AFx;
      HAL_GPIO_Init(USARTx_Tx_GPIO, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = USARTx_Rx_GPIO_PIN;
      HAL_GPIO_Init(USARTx_Rx_GPIO, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USARTx)
  {
      USART_RCC_CLK_DISABLE();

      /* 串口外设功能GPIO配置 */
      HAL_GPIO_DeInit(USARTx_Tx_GPIO, USARTx_Tx_GPIO_PIN);
      HAL_GPIO_DeInit(USARTx_Rx_GPIO, USARTx_Rx_GPIO_PIN);

      /* 串口中断禁用 */
      HAL_NVIC_DisableIRQ(USARTx_IRQn);
  }
}

void USARTx_IRQHANDLER(void)
{
    if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_RXNE)!= RESET)  // 接收中断：接收到数据
    {
        uint8_t data;
        data=READ_REG(husartx.Instance->DR); // 读取数据
        if(RxCount==0) // 如果是重新接收到数据帧，开启串口空闲中断
        {
            __HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE); // 清除空闲中断标志
          __HAL_UART_ENABLE_IT(&husartx,UART_IT_IDLE);     // 使能空闲中断
        }
        if(RxCount<RX_MAX_COUNT)    // 判断接收缓冲区未满
        {
            aRxBuffer[RxCount]=data;  // 保存数据
            RxCount++;                // 增加接收字节数计数
        }
    }
    else    if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_IDLE)!= RESET) // 串口空闲中断
    {
        __HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE); // 清除空闲中断标志
        __HAL_UART_DISABLE_IT(&husartx,UART_IT_IDLE);    // 关闭空闲中断
        Frame_flag=1;                                        // 数据帧置位，标识接收到一个完整数据帧
    }
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
