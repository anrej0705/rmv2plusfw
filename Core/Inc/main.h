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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void nema_start(void);
void nema_stop(void);
void nema_start_part(uint16_t length);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define nema_dis() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define nema_en() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

#define tens_en() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define tens_dis() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void config_speed(uint16_t intial_speed, uint16_t speed, uint16_t coef);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Emergency_Interrupt_Pin LL_GPIO_PIN_0
#define Emergency_Interrupt_GPIO_Port GPIOA
#define Tension_sensor_Pin LL_GPIO_PIN_1
#define Tension_sensor_GPIO_Port GPIOA
#define Camera_callback_Pin LL_GPIO_PIN_2
#define Camera_callback_GPIO_Port GPIOA
#define Keyb_scan_input_Pin LL_GPIO_PIN_4
#define Keyb_scan_input_GPIO_Port GPIOA
#define Keyb_scan_inputA5_Pin LL_GPIO_PIN_5
#define Keyb_scan_inputA5_GPIO_Port GPIOA
#define Keyb_scan_inputA6_Pin LL_GPIO_PIN_6
#define Keyb_scan_inputA6_GPIO_Port GPIOA
#define Keyb_scan_inputA7_Pin LL_GPIO_PIN_7
#define Keyb_scan_inputA7_GPIO_Port GPIOA
#define Keyb_lane_4_Pin LL_GPIO_PIN_4
#define Keyb_lane_4_GPIO_Port GPIOC
#define Keyb_lane_1_Pin LL_GPIO_PIN_0
#define Keyb_lane_1_GPIO_Port GPIOB
#define Keyb_lane_2_Pin LL_GPIO_PIN_1
#define Keyb_lane_2_GPIO_Port GPIOB
#define Keyb_lane_3_Pin LL_GPIO_PIN_2
#define Keyb_lane_3_GPIO_Port GPIOB
#define TTM_Dir_Pin LL_GPIO_PIN_10
#define TTM_Dir_GPIO_Port GPIOB
#define TTM_Drive_stepper_PWM_Pin LL_GPIO_PIN_14
#define TTM_Drive_stepper_PWM_GPIO_Port GPIOB
#define LD4_Pin LL_GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin LL_GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define PWM_Pulse_control_Pin LL_GPIO_PIN_12
#define PWM_Pulse_control_GPIO_Port GPIOA
#define TMS_SWDIO_Pin LL_GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TCK_SWCLK_Pin LL_GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define TTM_motion_sensor_Pin LL_GPIO_PIN_10
#define TTM_motion_sensor_GPIO_Port GPIOC
#define Roller_motion_sensor_Pin LL_GPIO_PIN_11
#define Roller_motion_sensor_GPIO_Port GPIOC
#define Bobin_motion_sensor_Pin LL_GPIO_PIN_12
#define Bobin_motion_sensor_GPIO_Port GPIOC
#define RGB_SPI_Pin LL_GPIO_PIN_3
#define RGB_SPI_GPIO_Port GPIOB
#define Tension_stepper_PWM_Pin LL_GPIO_PIN_4
#define Tension_stepper_PWM_GPIO_Port GPIOB
#define Film_sensor_Pin LL_GPIO_PIN_5
#define Film_sensor_GPIO_Port GPIOB
#define Tension_enable_Pin LL_GPIO_PIN_6
#define Tension_enable_GPIO_Port GPIOB
#define NEMA_EN_Pin LL_GPIO_PIN_7
#define NEMA_EN_GPIO_Port GPIOB
#define Camera_shutter_Pin LL_GPIO_PIN_8
#define Camera_shutter_GPIO_Port GPIOB
#define User_override_sensor_Pin LL_GPIO_PIN_9
#define User_override_sensor_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
