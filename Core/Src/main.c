/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body, Microstepping motor driver control in
 * half-stepping and microstepping modes (3/12/2023)
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ppr = 400;
float    pps;
float    td;
float    t;
float    pulse = 0;
float    pulse_require;
uint16_t PWM1 = 10000;
uint16_t PWM2 = 0;

uint8_t PinCodeHalfStep[] = {0x0E, 0x0A, 0x0B, 0x09, 0x0D, 0x05, 0x07, 0x06};
// uint8_t PinCodeOnePhase[] = {0x01,0x04,0x02,0x08};
// uint8_t PinCodeSecondPhase[] = {0x05,0x06,0x0A,0x09};
uint16_t StepUp[] =
  {980, 2000, 2900, 3800, 4700, 5600, 6300, 7100, 7700, 8300, 8800, 9300, 9560, 9800, 9950, 10000};
uint16_t StepDown[] =
  {9950, 9800, 9560, 9300, 8800, 8300, 7700, 7100, 6300, 5600, 4700, 3800, 2900, 2000, 980, 0};

// Helper: delay in microseconds using TIM2 counter
static void delay_us(uint32_t time_us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < time_us)
  {
    ; // busy wait
  }
}

static void startMicroStep(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

static void stopMicroStep(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

void MicroStep(float rpm, float angle)
{
  pulse = 0;
  pps   = (rpm * ppr) / 60.0f;
  td    = ((1.0f / pps) * 1000.0f);  // ms
  t     = (td / 16.0f) * 1000.0f;    // us
  pulse_require = angle * 128.0f / 7.2f;

  startMicroStep();

  while (pulse < pulse_require)
  {
    // for 1
    HAL_GPIO_WritePin(In1_GPIO_Port, In1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(In2_GPIO_Port, In2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_RESET);

    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, StepUp[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 2
    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, StepDown[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 3
    HAL_GPIO_WritePin(In1_GPIO_Port, In1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(In2_GPIO_Port, In2_Pin, GPIO_PIN_SET);

    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, StepUp[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 4
    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, StepDown[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 5
    HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_SET);

    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, StepUp[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 6
    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, StepDown[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 7
    HAL_GPIO_WritePin(In1_GPIO_Port, In1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(In2_GPIO_Port, In2_Pin, GPIO_PIN_RESET);

    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, StepUp[i]);
      delay_us(t);
      pulse = pulse + 1;
    }

    // for 8
    for (uint8_t i = 0; i < 16; i++)
    {
      if (pulse >= pulse_require)
        break;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, StepDown[i]);
      delay_us(t);
      pulse = pulse + 1;
    }
  }

  stopMicroStep();
}

void HalfStep(float rpm, float round)
{
  //
  pulse         = 0;
  pps           = (rpm * ppr) / 60.0f;
  td            = ((1.0f / pps) * 1000000.0f);
  pulse_require = (360 * round) * 8.0f / 7.2f;

  while (pulse < pulse_require)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 65535);
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 65535);

      HAL_GPIO_WritePin(In1_GPIO_Port, In1_Pin, (GPIO_PinState)(PinCodeHalfStep[i] & 0x01));
      HAL_GPIO_WritePin(In2_GPIO_Port, In2_Pin, (GPIO_PinState)(PinCodeHalfStep[i] & 0x02));
      HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, (GPIO_PinState)(PinCodeHalfStep[i] & 0x04));
      HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, (GPIO_PinState)(PinCodeHalfStep[i] & 0x08));

      delay_us(td);

      pulse = pulse + 1;
      if (pulse >= pulse_require)
        break;
    }
  }
}
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // HalfStep(50, 10);
  // MicroStep(10, 45);
  // MicroStep(10, 90);
  // MicroStep(10, 180);
  MicroStep(10, 3600);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 7;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 65535;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef      sConfigOC     = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 10000;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, In1_Pin | In4_Pin | In3_Pin | In2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin  = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA8 PA10 */
  GPIO_InitStruct.Pin =
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                        GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : In1_Pin In4_Pin In3_Pin In2_Pin */
  GPIO_InitStruct.Pin   = In1_Pin | In4_Pin | In3_Pin | In2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
