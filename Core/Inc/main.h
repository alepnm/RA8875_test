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
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_crc.h"
#include "stm32f4xx_ll_dac.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "stm32f4xx_hal_conf.h" //<-- reikalingas del teisingo taktavimo
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TOUCH_EVENT_FLAG    0x01
#define EVENT1_FLAG         0x02
#define EVENT2_FLAG         0x04
#define EVENT3_FLAG         0x08
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
__STATIC_INLINE void Delay_us(uint16_t delay)
{
    LL_TIM_ClearFlag_UPDATE(TIM7);
    LL_TIM_SetCounter(TIM7, 65535 - (delay - 1));
    while( !LL_TIM_IsActiveFlag_UPDATE(TIM7) );
}
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM1_Pin LL_GPIO_PIN_5
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin LL_GPIO_PIN_6
#define PWM2_GPIO_Port GPIOE
#define DS_DATA_Pin LL_GPIO_PIN_13
#define DS_DATA_GPIO_Port GPIOC
#define LCD_WAIT_Pin LL_GPIO_PIN_6
#define LCD_WAIT_GPIO_Port GPIOA
#define LCD_RST_Pin LL_GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOA
#define U4DE_Pin LL_GPIO_PIN_2
#define U4DE_GPIO_Port GPIOB
#define PWM3_Pin LL_GPIO_PIN_14
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin LL_GPIO_PIN_15
#define PWM4_GPIO_Port GPIOB
#define SD_DETECT_Pin LL_GPIO_PIN_6
#define SD_DETECT_GPIO_Port GPIOC
#define LD7_Pin LL_GPIO_PIN_9
#define LD7_GPIO_Port GPIOA
#define TEST_OUT_Pin LL_GPIO_PIN_15
#define TEST_OUT_GPIO_Port GPIOA
#define U2DE_Pin LL_GPIO_PIN_3
#define U2DE_GPIO_Port GPIOD
#define LCD_BACKLIGHT_Pin LL_GPIO_PIN_9
#define LCD_BACKLIGHT_GPIO_Port GPIOB
#define RA8875_INT_Pin LL_GPIO_PIN_0
#define RA8875_INT_GPIO_Port GPIOE
#define RA8875_INT_EXTI_IRQn EXTI0_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
