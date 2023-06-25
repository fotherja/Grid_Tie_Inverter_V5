/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ***************************************************************************************
 * MIT License
 * Copyright (c) [2023] [James Fotherby]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ***************************************************************************************/
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
#include "PR.h"
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
const float Sin_LookupF[65] =
{
	0.000000,0.098017,0.195090,0.290285,0.382683,0.471397,0.555570,0.634393,
	0.707107,0.773010,0.831470,0.881921,0.923880,0.956940,0.980785,0.995185,
	1.000000,0.995185,0.980785,0.956940,0.923880,0.881921,0.831470,0.773010,
	0.707107,0.634393,0.555570,0.471397,0.382683,0.290285,0.195090,0.098017,
	0.000000,-0.098017,-0.195090,-0.290285,-0.382683,-0.471397,-0.555570,-0.634393,
	-0.707107,-0.773010,-0.831470,-0.881921,-0.923880,-0.956940,-0.980785,-0.995185,
	-1.000000,-0.995185,-0.980785,-0.956940,-0.923880,-0.881921,-0.831470,-0.773010,
	-0.707107,-0.634393,-0.555570,-0.471397,-0.382683,-0.290285,-0.195090,-0.098017,
	0.000000
};

// These values are scaled between 0-256 to improve the integer maths in our PLL function.
const float Cos_LookupF[65] =
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

// Global Structures
PIDControl 						PLL_PID, I_OUT_PID, P_OUT_PID;
DFSDM_Filter_AwdParamTypeDef 	awdParamFilter0, awdParamFilter1;
PR_t 							PR_50, PR_150, PR_250, PR_350, PR_450, PR_550;

// Necessary Global variables (since R/W occurs within ISRs):
int32_t 			I_grid_DMA, V_grid_DMA, I_cap_DMA, V_bus_DMA;							// These variables are updated automatically by the DFSDM and DMA peripherals
volatile uint8_t	HB_Enabled_Flag = false;
volatile int16_t 	Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;									// Keeps track of whether the grid is safe to connect to

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HB_Disable(void);
void HB_Enable(void);
void Grid_Checks(void);
void Controller(void);
void PLL(void);
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

  /* Reset of all peripherals, Initialises the Flash interface and the Systick. */
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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // #######################################################################################
  // -------------------------------Initialisations-----------------------------------------
  // #######################################################################################

  // Initialise DAC for Debugging, ensure the relays are closed and ensure our H-Bridge driver is disabled
  HAL_GPIO_WritePin(GPIOA, Driver_Disable_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, Relay_Pin, GPIO_PIN_SET);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  // Initialise the PID controllers
  PIDInit(&PLL_PID, PLL_Kp, PLL_Ki, PLL_Kd, PLL_PERIOD, -PLL_LIMIT, PLL_LIMIT, AUTOMATIC, DIRECT, P_ON_E);
  PIDInit(&I_OUT_PID, I_OUT_Kp, I_OUT_Ki, I_OUT_Kd, I_OUT_PERIOD, -I_OUT_Limit, I_OUT_Limit, AUTOMATIC, DIRECT, P_ON_E);
  PIDInit(&P_OUT_PID, P_OUT_Kp, P_OUT_Ki, P_OUT_Kd, P_OUT_PERIOD, P_OUT_MIN, P_OUT_MAX, AUTOMATIC, REVERSE, P_ON_E);

  // Initialise our timers
  HAL_TIM_Base_Start_IT(&htim2);															// Used to periodically run the grid checks
  HAL_TIM_Base_Start_IT(&htim3);															// Used to periodically run the current controller
  HAL_TIM_Base_Start_IT(&htim4);															// Used to periodically run the PLL routine
  HAL_TIM_Base_Start_IT(&htim5);															// Used as a phase counter 0-63 per 50Hz period

  // Start the DFSDMs
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, &V_bus_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, &I_grid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter2, &V_grid_DMA, 1);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter3, &I_cap_DMA, 1);

  // Enable the interrupts by the analogue watchdog of the DFSDM peripheral (These are vitally important)
  awdParamFilter0.DataSource = DFSDM_FILTER_AWD_CHANNEL_DATA;
  awdParamFilter0.Channel = DFSDM_CHANNEL_0;
  awdParamFilter0.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.HighThreshold = 3700 << 8;												//420 = 50V, 1540 = 185V, 3700 = 445V
  awdParamFilter0.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter0.LowThreshold = -50 << 8;
  HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter0, &awdParamFilter0);

  awdParamFilter1.DataSource = DFSDM_FILTER_AWD_CHANNEL_DATA;
  awdParamFilter1.Channel = DFSDM_CHANNEL_1;
  awdParamFilter1.HighBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.HighThreshold = 2500 << 8;												//It's about 500 per A
  awdParamFilter1.LowBreakSignal = DFSDM_NO_BREAK_SIGNAL;
  awdParamFilter1.LowThreshold = -2500 << 8;
  HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter1, &awdParamFilter1);

  // Enable the interrupts by the analogue watchdog
  HAL_NVIC_EnableIRQ(DFSDM1_FLT0_IRQn);
  HAL_NVIC_EnableIRQ(DFSDM1_FLT1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // #######################################################################################
  // ----------------------------------Main loop--------------------------------------------
  // #######################################################################################
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* All code is handled by ISRs. Simply output to DAC for debugging */
	float I_grid = ((float)I_grid_DMA) * I_GRID_SENSOR_K;
	uint32_t DAC_Data = 2048 + (int32_t)(I_grid*421.0);										// This results in 3 A/V on our scope allowing for +/- 5A.

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_Data);
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
// #########################################################################################
// ---------------------------------Subroutines---------------------------------------------
// #########################################################################################
void HB_Disable() {
	// Disable the H-Bridge
	HAL_GPIO_WritePin(GPIOA, Driver_Disable_Pin, GPIO_PIN_SET);

	// Disable further interrupts by the analogue watchdog
	HAL_DFSDM_FilterAwdStop_IT(&hdfsdm1_filter0);
	HAL_DFSDM_FilterAwdStop_IT(&hdfsdm1_filter1);

	// Stop the PWM outputs
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	// Turn off the "H-Brige enabled LED", reset our grid check recovery period etc
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
	Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
	HB_Enabled_Flag = false;
}

void HB_Enable() {
	// Ensure we never proceed without ensuring our over current and voltage protection is enabled
	if((hdfsdm1_filter0.Instance->FLTCR2 & DFSDM_FLTCR2_AWDIE) == 0)	{
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		HB_Enabled_Flag = false;
		return;
	}
	if((hdfsdm1_filter1.Instance->FLTCR2 & DFSDM_FLTCR2_AWDIE) == 0)	{
		Grid_Good_Bad_Cnt = GRID_UNACCEPTABLE;
		HB_Enabled_Flag = false;
		return;
	}

	// Start our PWM driver which uses Timer1 to power our H-bridge. Both channels start with Duty = 0
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	// Enable the H-Bridge
	HAL_GPIO_WritePin(GPIOA, Driver_Disable_Pin, GPIO_PIN_RESET);
	HB_Enabled_Flag = true;

	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
}

float Integral(int32_t datum)	{
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

int32_t Integrate_Mains_MS(int32_t grid_voltage_sample)	{
	// Maintain an Mean squared measurement of the grid voltage. Runs at 800Hz which is 16 samples per period
	static int32_t 		Buffer[RMS_INTEGRAL_SIZE];
	static int8_t  		Index = 0;
	static int32_t  	Sum = 0;

	int32_t Sample_sqrd = grid_voltage_sample * grid_voltage_sample;
	Sum += Sample_sqrd;
	Buffer[Index++] = Sample_sqrd;

	if(Index == RMS_INTEGRAL_SIZE)
		Index = 0;

	Sum -= Buffer[Index];

	int32_t Mean_Squared = Sum >> 6;														// Divide Sum by 64 (RMS_INTEGRAL_SIZE)
	return(Mean_Squared);
}

void Grid_Checks()	{
	float 	V_grid 		= ((float)V_grid_DMA) * V_GRID_SENSOR_K;
	float 	V_bus 		= ((float)V_bus_DMA) * V_BUS_SENSOR_K;

	float 	Freq_Offset = PLL_PID.output * F_CONVERSION_K; 									// Get the frequency difference between the grid and a 50Hz reference
	int32_t Mains_MS 	= Integrate_Mains_MS((int32_t)V_grid); 								// Update our mains RMS measurement

	// These checks have to be out of range for an amount of time (<100ms)
	if(Mains_MS > RMS_UPPER_LIMIT || Mains_MS < RMS_LOWER_LIMIT)							// If grid Mean-Squared voltage out of tolerance
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;

	if(Freq_Offset > FREQ_DEV_LIMIT || Freq_Offset < -FREQ_DEV_LIMIT)						// If our mains frequency is out of tolerance
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;

	if(V_bus > V_BUS_MAXIMUM || V_bus < V_BUS_MINIMUM)										// If our DC Bus voltage is out of tolerance
		Grid_Good_Bad_Cnt -= GRID_BAD_FAIL_RATE;

	// If our grid checks fail we disconnect
	if(Grid_Good_Bad_Cnt < GRID_OK)	{														// If our metrics have been wrong for too long -> stop output
		if(HB_Enabled_Flag == true)
			HB_Disable(); 																	// This puts the H-bridge into a high impedance state
	}

	// If grid checks are looking good and we're not yet grid tied then request to join
	if(HB_Enabled_Flag == false) {
		if(Grid_Good_Bad_Cnt == GRID_OK)	{												// When grid is looking OK start our over V & I detection
			HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter1, &awdParamFilter1);				// Enable the interrupts by the analogue watchdog
			HAL_DFSDM_FilterAwdStart_IT(&hdfsdm1_filter0, &awdParamFilter0);
		}

		if(Grid_Good_Bad_Cnt == GRID_ACCEPTABLE && ENABLE_JOINING_GRID == true)				// If the grid remains good then request a join
			HB_Enabled_Flag = REQUEST_JOIN_GRID;
	}

	//-------------------------------------------------------------------------------
	Grid_Good_Bad_Cnt++;																	// If all checks are nominal decay our error metric
	Grid_Good_Bad_Cnt = CONSTRAIN(Grid_Good_Bad_Cnt, GRID_UNACCEPTABLE, GRID_ACCEPTABLE);

	// Here we adjust our output current to keep our bus voltage at 370V using a PI controller
	P_OUT_PID.setpoint = 370.0f;
	P_OUT_PID.input = V_bus;
	PIDCompute(&P_OUT_PID);
}

void Controller()	{
	// This enables our H-Bridge at the up-going zero crossing point if we are requesting to join the grid.
	if(HB_Enabled_Flag == REQUEST_JOIN_GRID)	{
		if(TIM5->CNT == 0)	{																// TIM5-CNT is our PLL phase counter running from 0->63
			I_OUT_PID.iTerm = 0;
			P_OUT_PID.iTerm = 0;
			P_OUT_PID.output = P_OUT_MIN;

			pr_init(&PR_50, 0.0f, 1000.0f, 10.0f, I_OUT_PERIOD);
			pr_init(&PR_150, 0.0f, 1500.0f, 10.0f, I_OUT_PERIOD);
			pr_init(&PR_250, 0.0f, 1000.0f, 10.0f, I_OUT_PERIOD);
			pr_init(&PR_350, 0.0f, 1000.0f, 10.0f, I_OUT_PERIOD);
			pr_init(&PR_450, 0.0f, 1000.0f, 10.0f, I_OUT_PERIOD);
			pr_init(&PR_550, 0.0f, 1000.0f, 10.0f, I_OUT_PERIOD);
			HB_Enable();
		}
	}

	// Interpolate our LO steps to get a more accurate LO sample:
	uint32_t Timer4_CNT = TIM4->CNT;														// Take recordings so they are less likely to change during the calculations
	uint8_t Lookup_Index = TIM5->CNT;
	float diff = Sin_LookupF[Lookup_Index + 1] - Sin_LookupF[Lookup_Index];					// This is why our lookup table has 65 values.
	float Timer4_CNT_Ratio = 0.001f * (float)Timer4_CNT;									// Timer4 counts from 0-999 (multiplication faster than division)
	float LO_Sample = Sin_LookupF[Lookup_Index] + (diff * Timer4_CNT_Ratio);

	// -------------- Update our current metrics:
	float I_grid = ((float)I_grid_DMA) * I_GRID_SENSOR_K;

	// -------------- Iterate our PI Controller:
	I_OUT_PID.setpoint = LO_Sample * P_OUT_PID.output;
	I_OUT_PID.input = I_grid;
	PIDCompute(&I_OUT_PID);

	// -------------- Iterate our Resonant Controllers:
	float Ui_50 = pr_calc(&PR_50, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 50.0);
	float Ui_150 = pr_calc(&PR_150, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 150.0);
	float Ui_250 = pr_calc(&PR_250, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 250.0);
	float Ui_350 = pr_calc(&PR_350, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 350.0);
	float Ui_450 = pr_calc(&PR_450, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 450.0);
	float Ui_550 = pr_calc(&PR_550, I_OUT_PID.setpoint, I_grid, 2 * 3.1415926 * 550.0);

	// -------------- Iterate our Feed-forward Controller:
	float Feedforward = LO_Sample * 358.0;

	// -------------- Add together the various control outputs:
	float Demanded_Output_Voltage = Feedforward + I_OUT_PID.output + Ui_50 + Ui_150 + Ui_250 + Ui_350 + Ui_450 + Ui_550;

	// -------------- Update our voltage metric:
	float V_bus = ((float)V_bus_DMA) * V_BUS_SENSOR_K;
	int16_t Duty_Cycle = (int16_t)(Demanded_Output_Voltage * DUTY_MAX / V_bus);

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
}

void PLL()	{
	float V_grid = ((float)V_grid_DMA) * V_GRID_SENSOR_K;
	int32_t Signal_Multiple = (int32_t)(Cos_LookupF[TIM5->CNT] * V_grid);					// Multiply the LO Cosine value with our current phase voltage sample
	PLL_PID.input = Integral(Signal_Multiple);   											// Integrate this Multiple over the last 1 period
	PIDCompute(&PLL_PID); 																	// Plug result in a PI controller to maintain 0 phase shift
	TIM4->ARR = SINE_STEP_PERIOD + (int32_t)PLL_PID.output; 								// adjust LO frequency (step period) to synchronise to grid (Important to have ARR_Preload enabled)
}

// #########################################################################################
// ----------------------------Interrupt Service Routines-----------------------------------
// #########################################################################################
void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold)	{
	HB_Disable();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3)																		// ## Runs at 13.3kHz to iterate the controller ##
		Controller();

	if (htim == &htim4)																		// ## Runs 3.2kHz and iterates the PLL ##
		PLL();

	if (htim == &htim2)																		// ## Runs 800Hz to perform our low priority safety checks ##
		Grid_Checks();
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
	HB_Disable();
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
