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
#define PWM_A_L_Pin GPIO_PIN_7
#define PWM_A_L_GPIO_Port GPIOA
#define PWM_B_L_Pin GPIO_PIN_0
#define PWM_B_L_GPIO_Port GPIOB
#define Vbus_DIN_Pin GPIO_PIN_1
#define Vbus_DIN_GPIO_Port GPIOB
#define Igrid_DIN_Pin GPIO_PIN_12
#define Igrid_DIN_GPIO_Port GPIOB
#define Vgrid_DIN_Pin GPIO_PIN_14
#define Vgrid_DIN_GPIO_Port GPIOB
#define Icap_DIN_Pin GPIO_PIN_7
#define Icap_DIN_GPIO_Port GPIOC
#define PWM_A_H_Pin GPIO_PIN_8
#define PWM_A_H_GPIO_Port GPIOA
#define PWM_B_H_Pin GPIO_PIN_9
#define PWM_B_H_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_11
#define Relay_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define 		CONSTRAIN(x,lower,upper) ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
//##############################################
#define 		PLL_Kp 					2.5e-5f 								// PID parameters for our PLL tracking control
#define 		PLL_Ki 					5.0e-6f
#define 		PLL_Kd 					0.0f
#define 		PLL_LIMIT 				50.0f
#define 		PLL_PERIOD 				3.125e-4f 								// 1 / (50 * 64) seconds
#define 		F_CONVERSION_K 			-5.0e-2f
//##############################################
#define			I_OUT_Kp				20.0f									// PID parameters for our current output controller
#define			I_OUT_Ki				10.0f
#define			I_OUT_Kd				0.0f
#define			I_OUT_Limit				400.0f									// Max voltage swing
#define			I_OUT_PERIOD			75.0e-6f
//##############################################
#define			P_OUT_Kp				2.5e-2f									// PID parameters for our power output controller
#define			P_OUT_Ki				2.5e-1f
#define			P_OUT_Kd				0.0f
#define			P_OUT_PERIOD			1.25e-3f
#define			P_OUT_MIN				0.5										// Peak output current controller limits
#define			P_OUT_MAX				4.0
//##############################################
#define 		SINE_STEP_PERIOD 		999 									// Ticks between incrementing our LO (Local Osc) index for 50Hz sine
#define 		DUTY_LIMIT 				975 									// Our duty width can vary from -1000 to +1000
#define			DUTY_MAX				1000.0f									// should be 999 but 1000 helps account for deadtime losses
#define			INTEGRAL_SIZE			64
#define			RMS_INTEGRAL_SIZE		64
#define 		GRID_ACCEPTABLE 		1000
#define 		GRID_OK					0
#define 		GRID_UNACCEPTABLE 		-1000
#define 		GRID_BAD_FAIL_RATE 		15 										// Some grid checks are allowed to fail for a certain amount of time (eg. Frequency) this parameter determines for how long
#define			REQUEST_JOIN_GRID		2
//##############################################
#define 		RMS_LOWER_LIMIT 		50000 									// These values are for the grid checks. We disconnect if our metrics are out of these ranges (SI Units)
#define 		RMS_UPPER_LIMIT 		70000									// Volts squared!
#define 		FREQ_DEV_LIMIT 			0.5f
#define			V_BUS_MINIMUM			360.0f
#define 		V_BUS_MAXIMUM 			425.0f

#define 		ENABLE_JOINING_GRID 	true

//##############################################
#define 		I_GRID_SENSOR_K 		-5.65e-8
#define 		V_BUS_SENSOR_K 			3.45e-6
#define 		V_GRID_SENSOR_K 		3.45e-6

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
