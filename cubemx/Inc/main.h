/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <rtthread.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct __Thread{
  rt_thread_t th;
  struct rt_semaphore sem;
  rt_mailbox_t mb;
  rt_timer_t tm;
}Thread;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
enum{
    WORK_STAT_FREE=0,
    WORK_STAT_MIS,                      //读卡，修改靶面
    WORK_STAT_IPAD_FREE,                //等待接收靶面状态
    WORK_STAT_IPAD_WORK,                //接收靶面工作状态
    WORK_STAT_INIT,                      //初始化获取IP
    WORK_STAT_CHARG,                     //充电插了typec
    WORK_STAT_CHARG_DONE,                //充电充满
    WORK_STAT_WAIT_RESET ,               //是否要重置
    WORK_STAT_RESET                     //重置IP 0x0000
};
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void rt_sec_delay(int sec);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
void key_th_entry(void *parameter);
void led_th_entry(void *parameter);
void rc522_th_entry(void *parameter);
void gc_send_th_entry(void *parameter);
void gc_recv_th_entry(void *parameter);
void gc_ioctl_th_entry(void*parameter);
void gc_chk_th_entry(void *parameter);
void gc_chk_tm_callback(void*parameter);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
