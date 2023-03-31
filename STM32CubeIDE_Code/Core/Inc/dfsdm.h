/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dfsdm.h
  * @brief   This file contains all the function prototypes for
  *          the dfsdm.c file
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
#ifndef __DFSDM_H__
#define __DFSDM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter2;

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter3;

extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DFSDM1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DFSDM_H__ */

