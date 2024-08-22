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
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.h>
#include "lcd_stm32f0.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 128        // Number of samples in LUT
#define TIM2CLK 8e6   // STM Clock frequency
#define F_SIGNAL 50 // Frequency of output analog signal - I think based off our circuit?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
uint32_t time_elapsed = 0;
uint32_t previous_time = 0;
uint32_t current_wave = 0;


// uint32_t Sin_LUT[NS] = {
// 		512,
// 		537.122649255638,
// 		562.184775848735,
// 		587.126002921145,
// 		611.886244872258,
// 		636.405852110471,
// 		660.625754754285,
// 		684.487604936817,
// 		707.933917370926,
// 		730.908207836304,
// 		753.355129254911,
// 		775.220605026929,
// 		796.451959306036,
// 		816.998043900126,
// 		836.809361491786,
// 		855.838184881673,
// 		874.038671967512,
// 		891.366976181739,
// 		907.781352121721,
// 		923.24225611809,
// 		937.712441498903,
// 		951.157048320139,
// 		963.543687346358,
// 		974.842518079203,
// 		985.026320645779,
// 		994.070561373707,
// 		1001.95345189489,
// 		1008.65600163561,
// 		1014.16206356645,
// 		1018.45837310197,
// 		1021.53458005616,
// 		1023.38327357705,
// 		1024,
// 		1023.38327357705,
// 		1021.53458005616,
// 		1018.45837310197,
// 		1014.16206356645,
// 		1008.65600163561,
// 		1001.95345189489,
// 		994.070561373707,
// 		985.026320645779,
// 		974.842518079203,
// 		963.543687346358,
// 		951.157048320139,
// 		937.712441498903,
// 		923.24225611809,
// 		907.781352121721,
// 		891.366976181739,
// 		874.038671967512,
// 		855.838184881673,
// 		836.809361491786,
// 		816.998043900126,
// 		796.451959306036,
// 		775.22060502693,
// 		753.355129254911,
// 		730.908207836304,
// 		707.933917370926,
// 		684.487604936817,
// 		660.625754754285,
// 		636.405852110471,
// 		611.886244872258,
// 		587.126002921145,
// 		562.184775848735,
// 		537.122649255638,
// 		512,
// 		486.877350744362,
// 		461.815224151265,
// 		436.873997078855,
// 		412.113755127742,
// 		387.594147889529,
// 		363.374245245715,
// 		339.512395063183,
// 		316.066082629074,
// 		293.091792163696,
// 		270.644870745089,
// 		248.779394973071,
// 		227.548040693964,
// 		207.001956099874,
// 		187.190638508214,
// 		168.161815118327,
// 		149.961328032488,
// 		132.633023818261,
// 		116.218647878279,
// 		100.75774388191,
// 		86.2875585010968,
// 		72.8429516798607,
// 		60.4563126536423,
// 		49.1574819207971,
// 		38.9736793542213,
// 		29.9294386262933,
// 		22.0465481051091,
// 		15.3439983643935,
// 		9.83793643354608,
// 		5.54162689803218,
// 		2.46541994383517,
// 		0.616726422951729,
// 		0,
// 		0.616726422951729,
// 		2.46541994383517,
// 		5.54162689803218,
// 		9.83793643354602,
// 		15.3439983643935,
// 		22.046548105109,
// 		29.9294386262933,
// 		38.9736793542212,
// 		49.157481920797,
// 		60.4563126536422,
// 		72.8429516798606,
// 		86.2875585010967,
// 		100.75774388191,
// 		116.218647878279,
// 		132.633023818261,
// 		149.961328032488,
// 		168.161815118326,
// 		187.190638508213,
// 		207.001956099874,
// 		227.548040693964,
// 		248.77939497307,
// 		270.644870745089,
// 		293.091792163695,
// 		316.066082629074,
// 		339.512395063183,
// 		363.374245245715,
// 		387.594147889529,
// 		412.113755127742,
// 		436.873997078854,
// 		461.815224151265,
// 		486.877350744362,
// 		512
// };
// //#endregion

// //# SAWTOOTH LUT
// uint32_t saw_LUT[NS] = {
// 		0,
// 		7.9921875,
// 		15.984375,
// 		23.9765625,
// 		31.96875,
// 		39.9609375,
// 		47.953125,
// 		55.9453125,
// 		63.9375,
// 		71.9296875,
// 		79.921875,
// 		87.9140625,
// 		95.90625,
// 		103.8984375,
// 		111.890625,
// 		119.8828125,
// 		127.875,
// 		135.8671875,
// 		143.859375,
// 		151.8515625,
// 		159.84375,
// 		167.8359375,
// 		175.828125,
// 		183.8203125,
// 		191.8125,
// 		199.8046875,
// 		207.796875,
// 		215.7890625,
// 		223.78125,
// 		231.7734375,
// 		239.765625,
// 		247.7578125,
// 		255.75,
// 		263.7421875,
// 		271.734375,
// 		279.7265625,
// 		287.71875,
// 		295.7109375,
// 		303.703125,
// 		311.6953125,
// 		319.6875,
// 		327.6796875,
// 		335.671875,
// 		343.6640625,
// 		351.65625,
// 		359.6484375,
// 		367.640625,
// 		375.6328125,
// 		383.625,
// 		391.6171875,
// 		399.609375,
// 		407.6015625,
// 		415.59375,
// 		423.5859375,
// 		431.578125,
// 		439.5703125,
// 		447.5625,
// 		455.5546875,
// 		463.546875,
// 		471.5390625,
// 		479.53125,
// 		487.5234375,
// 		495.515625,
// 		503.5078125,
// 		511.5,
// 		519.4921875,
// 		527.484375,
// 		535.4765625,
// 		543.46875,
// 		551.4609375,
// 		559.453125,
// 		567.4453125,
// 		575.4375,
// 		583.4296875,
// 		591.421875,
// 		599.4140625,
// 		607.40625,
// 		615.3984375,
// 		623.390625,
// 		631.3828125,
// 		639.375,
// 		647.3671875,
// 		655.359375,
// 		663.3515625,
// 		671.34375,
// 		679.3359375,
// 		687.328125,
// 		695.3203125,
// 		703.3125,
// 		711.3046875,
// 		719.296875,
// 		727.2890625,
// 		735.28125,
// 		743.2734375,
// 		751.265625,
// 		759.2578125,
// 		767.25,
// 		775.2421875,
// 		783.234375,
// 		791.2265625,
// 		799.21875,
// 		807.2109375,
// 		815.203125,
// 		823.1953125,
// 		831.1875,
// 		839.1796875,
// 		847.171875,
// 		855.1640625,
// 		863.15625,
// 		871.1484375,
// 		879.140625,
// 		887.1328125,
// 		895.125,
// 		903.1171875,
// 		911.109375,
// 		919.1015625,
// 		927.09375,
// 		935.0859375,
// 		943.078125,
// 		951.0703125,
// 		959.0625,
// 		967.0546875,
// 		975.046875,
// 		983.0390625,
// 		991.03125,
// 		999.0234375,
// 		1007.015625,
// 		1015.0078125,
// 		0
// };
// //#endregion

// //#region TRIANGULAR LUT
// uint32_t triangle_LUT[NS] = {
// 		0,
// 		15.984375,
// 		31.96875,
// 		47.953125,
// 		63.9375,
// 		79.921875,
// 		95.90625,
// 		111.890625,
// 		127.875,
// 		143.859375,
// 		159.84375,
// 		175.828125,
// 		191.8125,
// 		207.796875,
// 		223.78125,
// 		239.765625,
// 		255.75,
// 		271.734375,
// 		287.71875,
// 		303.703125,
// 		319.6875,
// 		335.671875,
// 		351.65625,
// 		367.640625,
// 		383.625,
// 		399.609375,
// 		415.59375,
// 		431.578125,
// 		447.5625,
// 		463.546875,
// 		479.53125,
// 		495.515625,
// 		511.5,
// 		527.484375,
// 		543.46875,
// 		559.453125,
// 		575.4375,
// 		591.421875,
// 		607.40625,
// 		623.390625,
// 		639.375,
// 		655.359375,
// 		671.34375,
// 		687.328125,
// 		703.3125,
// 		719.296875,
// 		735.28125,
// 		751.265625,
// 		767.25,
// 		783.234375,
// 		799.21875,
// 		815.203125,
// 		831.1875,
// 		847.171875,
// 		863.15625,
// 		879.140625,
// 		895.125,
// 		911.109375,
// 		927.09375,
// 		943.078125,
// 		959.0625,
// 		975.046875,
// 		991.03125,
// 		1007.015625,
// 		1023,
// 		1007.015625,
// 		991.03125,
// 		975.046875,
// 		959.0625,
// 		943.078125,
// 		927.09375,
// 		911.109375,
// 		895.125,
// 		879.140625,
// 		863.15625,
// 		847.171875,
// 		831.1875,
// 		815.203125,
// 		799.21875,
// 		783.234375,
// 		767.25,
// 		751.265625,
// 		735.28125,
// 		719.296875,
// 		703.3125,
// 		687.328125,
// 		671.34375,
// 		655.359375,
// 		639.375,
// 		623.390625,
// 		607.40625,
// 		591.421875,
// 		575.4375,
// 		559.453125,
// 		543.46875,
// 		527.484375,
// 		511.5,
// 		495.515625,
// 		479.53125,
// 		463.546875,
// 		447.5625,
// 		431.578125,
// 		415.59375,
// 		399.609375,
// 		383.625,
// 		367.640625,
// 		351.65625,
// 		335.671875,
// 		319.6875,
// 		303.703125,
// 		287.71875,
// 		271.734375,
// 		255.75,
// 		239.765625,
// 		223.78125,
// 		207.796875,
// 		191.8125,
// 		175.828125,
// 		159.84375,
// 		143.859375,
// 		127.875,
// 		111.890625,
// 		95.90625,
// 		79.921875,
// 		63.9375,
// 		47.953125,
// 		31.96875,
// 		15.984375,
// 		0
// };
uint32_t Sin_LUT[NS] =
    {511, 536, 562, 587, 612, 636, 661, 685, 708, 731, 754, 776, 797, 818, 838, 857, 875, 892, 909, 924, 938,
     952, 964, 975, 985, 994, 1002, 1008, 1014, 1018, 1021, 1022, 1022, 1022, 1019, 1016, 1011, 1005, 998, 990,
     980, 970, 958, 945, 931, 916, 901, 884, 866, 847, 828, 808, 787, 765, 743, 720, 696, 673, 648, 624, 599,
     574, 549, 524, 498, 473, 448, 423, 398, 374, 349, 326, 302, 279, 257, 235, 214, 194, 175, 156, 138, 121,
     106, 91, 77, 64, 52, 42, 32, 24, 17, 11, 6, 3, 0, 0, 0, 1, 4, 8, 14, 20, 28, 37, 47, 58, 70, 84, 98, 113,
     130, 147, 165, 184, 204, 225, 246, 268, 291, 314, 337, 361, 386, 410, 435, 460, 486, 511};
uint32_t saw_LUT[NS] =
    {0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128, 136, 144, 153, 161, 169, 177, 185,
     193, 201, 209, 217, 225, 233, 241, 249, 257, 265, 273, 281, 289, 298, 306, 314, 322, 330, 338, 346, 354,
     362, 370, 378, 386, 394, 402, 410, 418, 426, 434, 443, 451, 459, 467, 475, 483, 491, 499, 507, 515, 523,
     531, 539, 547, 555, 563, 571, 579, 588, 596, 604, 612, 620, 628, 636, 644, 652, 660, 668, 676, 684, 692,
     700, 708, 716, 724, 733, 741, 749, 757, 765, 773, 781, 789, 797, 805, 813, 821, 829, 837, 845, 853, 861,
     869, 878, 886, 894, 902, 910, 918, 926, 934, 942, 950, 958, 966, 974, 982, 990, 998, 1006, 1014, 1023};
uint32_t triangle_LUT[NS] =
    {0, 16, 32, 48, 65, 81, 97, 113, 129, 145, 161, 177, 193, 209, 225, 241,
     257, 273, 289, 305, 321, 337, 353, 369, 385, 401, 417, 433, 449, 465, 481, 497,
     513, 529, 545, 561, 577, 593, 609, 625, 641, 657, 673, 689, 705, 721, 737, 753,
     769, 785, 801, 817, 833, 849, 865, 881, 897, 913, 929, 945, 961, 977, 993, 1009,
     1023, 1009, 993, 977, 961, 945, 929, 913, 897, 881, 865, 849, 833, 817, 801, 785,
     769, 753, 737, 721, 705, 689, 673, 657, 641, 625, 609, 593, 577, 561, 545, 529,
     513, 497, 481, 465, 449, 433, 417, 401, 385, 369, 353, 337, 321, 305, 289, 273,
     257, 241, 225, 209, 193, 177, 161, 145, 129, 113, 97, 81, 65, 48, 32, 16};

// TODO: Equation to calculate TIM2_Ticks

uint32_t TIM2_Ticks = TIM2CLK/(F_SIGNAL*NS); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) & (TIM3->CCR3);  // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  char clockspeed = HAL_RCC_GetSysClockFreq(); // gets the current set clock speed of the chip
  printf(clockspeed);
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS);
  // feeds the LUT values into the PWM generator tim3 based on tim2 timing

  delay(3000);

  // TODO: Write current waveform to LCD ("Sine")
  // current_wave = 0;
  lcd_command(CLEAR);
  lcd_putstring("Sine");

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }
  LL_SetSystemCoreClock(8000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //Change period to time between LUT values
  htim2.Init.Period = TIM2_Ticks-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  //
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  //
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  //
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  //
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
  

  // TODO: Debounce using HAL_GetTick()
  time_elapsed = HAL_GetTick();
  if (time_elapsed - previous_time > 500)
  {
    // TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
    // HINT: Consider using C's "switch" function to handle LUT changes
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
    HAL_DMA_Abort_IT(&hdma_tim2_ch1);
    lcd_command(CLEAR);
    if (current_wave == 2)
      current_wave = 0;
    else
      current_wave++;
    switch (current_wave)
    {
      
    case 0:
      
      lcd_putstring("Sine");
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) Sin_LUT, DestAddress, NS);

      break;
    case 1:
      
      lcd_putstring("Sawtooth");
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) saw_LUT, DestAddress, NS);
      break;
    case 2:
      
      lcd_putstring("Triangle");
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) triangle_LUT, DestAddress, NS);
      break;
    default:
      lcd_putstring("An error has occured with the wave counter");
      break;
    }

    
    // Enable DMA (start transfer from LUT to CCR)
    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
    previous_time = time_elapsed;

    
  }

 

  HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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

#ifdef USE_FULL_ASSERT
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