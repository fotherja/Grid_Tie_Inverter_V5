/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  *
  *	To Do:
  *	1) AWD for DFSDM cutouts
  *	2) Add 150, 250 and 350Hz Resonant terms
  *	3) Test PLL again and tune it for a 240V grid
  *	4) Flash LED when grid checks are not successful
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dfsdm.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Sin_LookupF[65] =
{
		0, 25.09238792, 49.94312244, 74.31287738, 97.96695869, 120.6775646, 142.2259797, 162.4046807,
		181.019336, 197.8906761, 212.8562207, 225.7718437, 236.5131603, 244.9767259, 251.0810318, 254.76729,
		256, 254.76729, 251.0810318, 244.9767259, 236.5131603, 225.7718437, 212.8562207, 197.8906761,
		181.019336,	162.4046807, 142.2259797, 120.6775646, 97.96695869, 74.31287738, 49.94312244, 25.09238792,
		0, -25.09238792, -49.94312244, -74.31287738, -97.96695869, -120.6775646, -142.2259797, -162.4046807,
		-181.019336, -197.8906761, -212.8562207, -225.7718437, -236.5131603, -244.9767259, -251.0810318, -254.76729,
		-256, -254.76729, -251.0810318, -244.9767259, -236.5131603, -225.7718437, -212.8562207, -197.8906761,
		-181.019336, -162.4046807, -142.2259797, -120.6775646, -97.96695869, -74.31287738, -49.94312244, -25.09238792,
		0
};

float Cos_LookupF[65] =
{
		256, 254.76729, 251.0810318, 244.9767259, 236.5131603, 225.7718437, 212.8562207, 197.8906761,
		181.019336, 162.4046807, 142.2259797, 120.6775646, 97.96695869, 74.31287738, 49.94312244, 25.09238792,
		0, -25.09238792, -49.94312244, -74.31287738, -97.96695869, -120.6775646, -142.2259797, -162.4046807,
		-181.019336, -197.8906761, -212.8562207, -225.7718437, -236.5131603, -244.9767259, -251.0810318, -254.76729,
		-256, -254.76729, -251.0810318, -244.9767259, -236.5131603, -225.7718437, -212.8562207, -197.8906761,
		-181.019336, -162.4046807, -142.2259797, -120.6775646, -97.96695869, -74.31287738, -49.94312244, -25.09238792,
		0, 25.09238792, 49.94312244, 74.31287738, 97.96695869, 120.6775646, 142.2259797, 162.4046807,
		181.019336,	197.8906761, 212.8562207, 225.7718437, 236.5131603, 244.9767259, 251.0810318, 254.76729,
		256
};

// PID Structures
PIDControl 			PLL_PID, I_OUT_PID; 										// These structures are used to store the respective PID variables

// DMA buffers and key values
int32_t 			Vbus_DMA[1], Igrid_DMA[1], Vgrid_DMA[1], Icap_DMA[1];
float 				I_grid, V_grid, I_cap, V_bus; 								// These are our SI unit values

// LO Phase
uint8_t 			_50_Index = 0;

volatile uint8_t 	Error_Code;
volatile float 		I_Output_Demand = 0;

volatile uint8_t    Run_Controller_Flag = 0;
volatile uint8_t	Run_Grid_Checks_Flag = 0;
uint8_t				HB_Enabled_Flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HB_Disable(void);
void HB_Enable(void);
void Grid_Checks(void);
void Controller(void);
void PLL(void);
void Main_Loop(void);
void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold);
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
float Integral(int32_t datum);
int32_t Integrate_Mains_MS(int32_t grid_voltage_sample);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_DFSDM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  // Initialise the PID controller for the PLL
  PIDInit(&PLL_PID, PLL_Kp, PLL_Ki, PLL_Kd, PLL_PERIOD, -PLL_LIMIT, PLL_LIMIT, AUTOMATIC, DIRECT, P_ON_E);
  PIDInit(&I_OUT_PID, I_OUT_Kp, I_OUT_Ki, I_OUT_Kd, I_OUT_PERIOD, -I_OUT_Limit, I_OUT_Limit, AUTOMATIC, DIRECT, P_ON_E);

  // Initialise our timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  // Initialise DAC and LED for Debugging
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOA, Relay_Pin, GPIO_PIN_SET);

  // Enable the interrupts by the analogue watchdog
  DFSDM_Filter_AwdParamTypeDef awdParamFilter0;
  awdParamFilter0.DataSource = DFSDM_FILTER_AWD_CHANNEL_DATA;
  awdParamFilter0.Channel = DFSDM_CHANNEL_0;
  awdParamFilter0.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.HighThreshold = 400 << 8;												//3600 << 8; about 400V
  awdParamFilter0.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.LowThreshold = -50 << 8;
  HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter0, &awdParamFilter0);

  DFSDM_Filter_AwdParamTypeDef awdParamFilter1;
  awdParamFilter1.DataSource = DFSDM_FILTER_AWD_CHANNEL_DATA;
  awdParamFilter1.Channel = DFSDM_CHANNEL_1;
  awdParamFilter1.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.HighThreshold = 2700 << 8;
  awdParamFilter1.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.LowThreshold = -2700 << 8;
  HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter1, &awdParamFilter1);

  // Start the DFSDMs
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, Vbus_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, Igrid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter2, Vgrid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter3, Icap_DMA, 1);

  // Enable the interrupts by the analogue watchdog
  HAL_NVIC_EnableIRQ(DFSDM1_FLT0_IRQn);
  HAL_NVIC_EnableIRQ(DFSDM1_FLT1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HB_Disable();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Main_Loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Main_Loop()	{
	if(Run_Controller_Flag == 1)	{
		Run_Controller_Flag = 0;
		Controller();
	}

	if(Run_Grid_Checks_Flag == 1)	{
		Run_Grid_Checks_Flag = 0;
		Grid_Checks();
	}
}

void HB_Disable() {
	// Disable the H-Bridge
	HAL_GPIO_WritePin(GPIOA, Driver_Disable_Pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HB_Enabled_Flag = false;

	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;

	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
}

void HB_Enable() {
	// Start our PWM driver which uses Timer1 to power our H-bridge. Both channels start with Duty = 0
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HB_Enabled_Flag = true;

	HAL_GPIO_WritePin(GPIOA, Driver_Disable_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
}

float Integral(int32_t datum)
{
	// Keep a running summation of the last INTEGRAL_SIZE values passed into this function.
	static int32_t 		Buffer[INTEGRAL_SIZE];
	static int16_t  	Index = 0;
	static int32_t  	Sum = 0;

	Sum += datum;

	Buffer[Index++] = datum;

	if(Index == INTEGRAL_SIZE)
		Index = 0;

	Sum -= Buffer[Index];
	return((float)Sum);
}

int32_t Integrate_Mains_MS(int32_t grid_voltage_sample)
{
	// Maintain an MS measurement of the grid voltage. Runs at 800Hz which is 16 samples per period
	static int32_t 		Buffer[RMS_INTEGRAL_SIZE];
	static int8_t  		Index = 0;
	static int32_t  	Sum = 0;

	int32_t Sample_sqrd = grid_voltage_sample * grid_voltage_sample;

	Sum += Sample_sqrd;

	Buffer[Index++] = Sample_sqrd;

	if(Index == RMS_INTEGRAL_SIZE)
		Index = 0;

	Sum -= Buffer[Index];

	int32_t Mean_Squared = Sum >> 6;			// Divide Sum by 64 (RMS_INTEGRAL_SIZE)
	return(Mean_Squared);						// return(sqrtf(RMS));
}

void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold)
{
	HB_Disable();
	HB_Enabled_Flag = OVER_IV_TRIP;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)	{
	if(hdfsdm_filter == &hdfsdm1_filter0)	{
		V_bus = (float)Vbus_DMA[0] * 2.4414e-7 + 5.0e-4;
	}
	if(hdfsdm_filter == &hdfsdm1_filter1)	{
		I_grid = (float)Igrid_DMA[0] * -3.96725e-9 + 8.75e-6;
	}
	if(hdfsdm_filter == &hdfsdm1_filter2)	{
		V_grid = (float)Vgrid_DMA[0] * 2.4414e-7 + 5.0e-4;
	}
	if(hdfsdm_filter == &hdfsdm1_filter3)	{
		I_cap = (float)Icap_DMA[0] * 3.1738e-9 - 8.75e-6;
	}
}

void Grid_Checks()	{
	static 	int16_t Grid_Good_Bad_Cnt 	= GRID_UNACCEPTABLE;
	int32_t 		Mains_MS 			= Integrate_Mains_MS((int32_t)V_grid); 		// Update our mains RMS measurement
	float 			Freq_Offset 		= PLL_PID.output * F_CONVERSION_K; 			// Get the frequency difference between the grid and a 50Hz reference

	// These are our high priority checks:
	if(V_bus > V_BUS_MAXIMUM)	{													// If our DC Bus voltage is too high cut-out immediately
		HB_Disable();
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		Error_Code |= 0b00001;
	}

	if(I_grid > I_OUTPUT_MAXIMUM || I_grid < -I_OUTPUT_MAXIMUM)	{					// Likewise, if we detect excess current flowing, cut-out immediately
		HB_Disable();
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		Error_Code |= 0b00010;
	}
	//------------------------------------------------------------------------------

	// These are lower priority checks and have to be out of range for an amount of time
	if(Mains_MS > RMS_UPPER_LIMIT || Mains_MS < RMS_LOWER_LIMIT)	{				// If mains RMS voltage whacky
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;
		Error_Code |= 0b00100;
	}

	if(Freq_Offset > FREQ_DEVIATION_LIMIT || Freq_Offset < -FREQ_DEVIATION_LIMIT)	{
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;									// If our mains frequency is out of tolerance
		Error_Code |= 0b01000;
	}

	if(V_bus < V_BUS_MINIMUM)	{													// If our DC Bus voltage is too low
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;
		Error_Code |= 0b10000;
	}
	//------------------------------------------------------------------------------

	// Act on the above checks
	if(Grid_Good_Bad_Cnt < GRID_OK)	{												// If our metrics have been whacky for too long stop output
		if(HB_Enabled_Flag == true) {
			HB_Disable(); 															// This puts the H-bridge into a high impedance state
			Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		}
	}

	if(HB_Enabled_Flag == false) { 													// With our output idle, if the grid has normalised, restart
		if(Grid_Good_Bad_Cnt == GRID_ACCEPTABLE && ENABLE_JOINING_GRID == true) {
			HB_Enabled_Flag = REQUEST_JOIN_GRID;
			Error_Code = 0;
		}
	}
	if(HB_Enabled_Flag == OVER_IV_TRIP)	{
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		HB_Enabled_Flag = false;
	}
	//-------------------------------------------------------------------------------

	// Constrain and decay any error counts
	Grid_Good_Bad_Cnt++;
	Grid_Good_Bad_Cnt = CONSTRAIN(Grid_Good_Bad_Cnt, GRID_UNACCEPTABLE, GRID_ACCEPTABLE);

	if(I_Output_Demand < 12.0e-3f)
		I_Output_Demand += 2.0e-6f;
}

void Controller()	{
	float const kt = 2.6666666e4;
	static int16_t Duty_Cycle;
	static float E_Prev1 = 0, E_Prev2 = 0;

	// 50 Hz Resonant term
	float const kr_50 = 4000.0;
	float const Wdamp_50 = 2.0 * 3.1415927 * 0.75;
	float const Wres_50  = 2.0 * 3.1415927 * 50.0;

	float const a1_50 = 2 * kr_50 * kt * Wdamp_50;
	float const b0_50 = kt * kt + 2 * kt * Wdamp_50 + Wres_50  * Wres_50 ;
	float const b1_50 = 2 * kt * kt - 2 * Wres_50  * Wres_50 ;
	float const b2_50 = kt * kt + 2 * kt * Wdamp_50 + Wres_50  * Wres_50 ;

	static float U_Prev1_50 = 0, U_Prev2_50 = 0;

	// 150 Hz Resonant term
	float const kr_150 = 1000.0;
	float const Wdamp_150 = 2.0 * 3.1415927 * 1.5;
	float const Wres_150  = 2.0 * 3.1415927 * 150.0;

	float const a1_150 = 2 * kr_150 * kt * Wdamp_150;
	float const b0_150 = kt * kt + 2 * kt * Wdamp_150 + Wres_150  * Wres_150 ;
	float const b1_150 = 2 * kt * kt - 2 * Wres_150  * Wres_150 ;
	float const b2_150 = kt * kt + 2 * kt * Wdamp_150 + Wres_150  * Wres_150 ;

	static float U_Prev1_150 = 0, U_Prev2_150 = 0;

	// This enables our H-Bridge at the up-going zero crossing point if we are requesting to join the grid.
	if(HB_Enabled_Flag == REQUEST_JOIN_GRID)	{
		if(_50_Index == 0)	{
			I_OUT_PID.iTerm = 0;
			E_Prev2 = 0; E_Prev1 = 0;												// Absolutely key to zero these! Perhaps why v4 exploded...
			U_Prev2_50 = 0; U_Prev1_50 = 0;
			I_Output_Demand = 4.0e-3f;
			HB_Enable();
		}
	}

	// Interpolate our LO steps to get a more accurate LO sample:
	uint32_t Timer4_CNT = TIM4->CNT;												// Take records of the current counter values
	uint8_t Lookup_Index = _50_Index;
	float diff = Sin_LookupF[Lookup_Index + 1] - Sin_LookupF[Lookup_Index];
	float Timer4_CNT_Ratio = 0.001f * (float)Timer4_CNT;							// Timer4 counts from 0-999 (multiplication faster than division)
	float LO_Sample_256 = Sin_LookupF[Lookup_Index] + (diff * Timer4_CNT_Ratio);

	// -------------- Iterate our PI Controller:
	// Calculations for the PI controller:
	I_OUT_PID.setpoint = LO_Sample_256 * I_Output_Demand;
	I_OUT_PID.input = I_grid;
	PIDCompute(&I_OUT_PID);

	// Calculations for the 50Hz resonant controller:
	float Ua_50 = a1_50 * E_Prev1 - a1_50 * E_Prev2;
	float Ub_50 = b1_50 * U_Prev1_50 - b2_50 * U_Prev2_50;
	float Ui_50 = (Ua_50 + Ub_50) / b0_50;

	U_Prev2_50 = U_Prev1_50;
	U_Prev1_50 = Ui_50;

	// Calculations for the 150Hz resonant controller:
	float Ua_150 = a1_150 * E_Prev1 - a1_150 * E_Prev2;
	float Ub_150 = b1_150 * U_Prev1_150 - b2_150 * U_Prev2_150;
	float Ui_150 = (Ua_150 + Ub_150) / b0_150;

	U_Prev2_150 = U_Prev1_150;
	U_Prev1_150 = Ui_150;

	// Record the previous error values then calculate our overall voltage to output
	E_Prev2 = E_Prev1;
	E_Prev1 = I_OUT_PID.error;

	float Demanded_Output_Voltage = I_OUT_PID.output + Ui_50 + Ui_150;
	Duty_Cycle = (int16_t)(Demanded_Output_Voltage * DUTY_MAX / V_bus);				//Duty_Cycle = 800; (int16_t)(LO_Sample_256 * 3.8);

	// Constrain and output the new duty cycle
	Duty_Cycle = CONSTRAIN(Duty_Cycle, -DUTY_LIMIT, DUTY_LIMIT);

	if(Duty_Cycle >= 0) {
		htim1.Instance->CCR1 = Duty_Cycle;
		htim1.Instance->CCR2 = 0;
	}
	else {
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = -Duty_Cycle;
	}

	/* Output to DAC for debugging */
	//uint32_t DAC_Data = 2048 + (int32_t)(Ui_150 * 100.0f);
	//uint32_t DAC_Data = 2048 + (int32_t)(LO_Sample_256*6.0);
	//uint32_t DAC_Data = 2048 + (int32_t)(V_grid * 100.0f);
	uint32_t DAC_Data = 2048 + (int32_t)(I_grid*316.0);								// This results in 4 A/V on our scope allowing for +/- 5A.
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_Data);
}

void PLL()	{
	int32_t Signal_Multiple = (int32_t)(Cos_LookupF[_50_Index] * V_grid);			// Multiply the LO Cosine value with our current phase voltage sample
	PLL_PID.input = Integral(Signal_Multiple);   									// Integrate this Multiple over the last 1 period
	PIDCompute(&PLL_PID); 															// Plug result in a PI controller to maintain 0 phase shift

	TIM4->ARR = SINE_STEP_PERIOD + (int32_t)PLL_PID.output; 						// adjust LO frequency (step period) to synchronise to grid

	_50_Index++;  																	// Increment our LO indices
	if(_50_Index == SINE_STEPS)
		_50_Index = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2)	{
		Run_Grid_Checks_Flag = 1;													// #### Runs 800Hz to perform our safety checks ####
	}

	if (htim == &htim3)	{
		Run_Controller_Flag = 1;													// ### Runs at 13.3kHz to iterate the controller ###
	}

	if (htim == &htim4)	{															// ### Runs 3.2 kHz and iterates the PLL ###
		PLL();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
