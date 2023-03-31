/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define DFSDM_Clk_Pin GPIO_PIN_2
#define DFSDM_Clk_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define Driver_Disable_Pin GPIO_PIN_2
#define Driver_Disable_GPIO_Port GPIOA
#define DAC_Out_Pin GPIO_PIN_5
#define DAC_Out_GPIO_Port GPIOA
#define Phase_BN_Pin GPIO_PIN_7
#define Phase_BN_GPIO_Port GPIOA
#define Phase_AN_Pin GPIO_PIN_0
#define Phase_AN_GPIO_Port GPIOB
#define Vbus_DIN_Pin GPIO_PIN_1
#define Vbus_DIN_GPIO_Port GPIOB
#define Igrid_DIN_Pin GPIO_PIN_12
#define Igrid_DIN_GPIO_Port GPIOB
#define Vgrid_DIN_Pin GPIO_PIN_14
#define Vgrid_DIN_GPIO_Port GPIOB
#define Icap_DIN_Pin GPIO_PIN_7
#define Icap_DIN_GPIO_Port GPIOC
#define Phase_B_Pin GPIO_PIN_8
#define Phase_B_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_9
#define Phase_A_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_11
#define Relay_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define 		PLL_Kp 					1.0e-3f 								// PID parameters for our PLL tracking control
#define 		PLL_Ki 					1.0e-4f									//
#define 		PLL_Kd 					0.0f
#define 		PLL_LIMIT 				50.0f
#define 		PLL_PERIOD 				3.125e-4f 								// 1 / (50 * 64) seconds
#define 		SINE_STEP_PERIOD 		999 									// Ticks between incrementing our LO (Local Osc) index for 50Hz sine
//##############################################
#define			I_OUT_Kp				5.0e0f									// PID parameters for our current output controller
#define			I_OUT_Ki				1.0e0f
#define			I_OUT_Kd				0.0f
#define			I_OUT_Limit				100.0f									// Max voltage swing
#define			I_OUT_PERIOD			75.0e-6f

//##############################################
#define 		SINE_STEPS           	64                          			// Number of steps to build our sine wave in
#define 		DUTY_LIMIT 				975 									// Our duty width can vary from -1000 to +1000
#define			DUTY_MAX				1000.0f
#define			INTEGRAL_SIZE			64
#define			RMS_INTEGRAL_SIZE		64

#define 		F_CONVERSION_K 			8.065e-3f								// These are calibration constants

#define 		RMS_LOWER_LIMIT 		0 										// These values are for the grid checks. We disconnect if our metrics are out of these ranges (SI Units)
#define 		RMS_UPPER_LIMIT 		62500
#define 		FREQ_DEVIATION_LIMIT 	0.5f
#define			V_BUS_MINIMUM			-5.0f
#define 		V_BUS_MAXIMUM 			380.0f
#define 		I_OUTPUT_MAXIMUM 		5.0f

#define 		ENABLE_JOINING_GRID 	true
#define			REQUEST_JOIN_GRID		2
#define			OVER_IV_TRIP			3

#define 		GRID_ACCEPTABLE 		1000
#define 		GRID_UNACCEPTABLE 		-1000
#define 		GRID_BAD_FAIL_RATE 		10 										// Some grid checks are allowed to fail for a certain amount of time (eg. Frequency) this parameter determines for how long
#define 		GRID_OK					0

#define 		CONSTRAIN(x,lower,upper) ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
