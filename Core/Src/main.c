/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 电机工作方向。 */
typedef enum
{
  MOTOR_STOP = 0,
  MOTOR_FORWARD,
  MOTOR_REVERSE
} MotorDirection;

/* 按键使用外部上拉，按下时读到低电平。 */
typedef struct
{
  uint16_t pin;
  GPIO_PinState stableState;
  GPIO_PinState lastRawState;
  uint32_t lastChangeTick;
} Button;

/* PB12：正转/反转切换；PB13：停止；PB14：加速；PB15：减速。 */
static Button buttonDirection = {GPIO_PIN_12, GPIO_PIN_SET, GPIO_PIN_SET, 0};
static Button buttonStop = {GPIO_PIN_13, GPIO_PIN_SET, GPIO_PIN_SET, 0};
static Button buttonSpeedUp = {GPIO_PIN_14, GPIO_PIN_SET, GPIO_PIN_SET, 0};
static Button buttonSpeedDown = {GPIO_PIN_15, GPIO_PIN_SET, GPIO_PIN_SET, 0};

static MotorDirection motorDirection = MOTOR_FORWARD; /* 上电默认正转。 */
static uint8_t motorDutyPercent = 20; /* 上电默认占空比 30%（占空比下限）。 */

#define MOTOR_DUTY_MIN       20U
#define MOTOR_DUTY_MAX       80U
#define MOTOR_DUTY_STEP      20U
#define BUTTON_DEBOUNCE_MS   20U

/* 返回当前占空比对应的定时器比较值。
 * PWM1 模式向上计数，CNT < CCR 时输出高电平，占空比 = CCR / (ARR + 1)。
 * 当前 ARR = 799（周期 800 个计数），CCR = 800 * dutyPercent / 100：
 *   30% -> 240，50% -> 400，70% -> 560，80% -> 640，均小于 800，波形正确。 */
static uint32_t Motor_GetCompareValue(uint8_t dutyPercent)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);
  return ((period + 1U) * dutyPercent) / 100U;
}

/* 换向时先关断两路 PWM，再更新桥臂状态，确保两路 PWM 不会同时导通。 */
static void Motor_SetDirection(MotorDirection direction)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U); /* PA1：左侧 HIN */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U); /* PA3：右侧 HIN */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); /* PA2 高：左侧 /LIN 关闭 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); /* 右侧 /LIN 关闭 */
  HAL_Delay(2U); /* 换向前短暂停顿。 */

  if (direction == MOTOR_FORWARD)
  {
    /* 正转：Q1 使用 PA1 PWM，Q4 导通；Q2、Q3 关闭。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); /* Q2 关闭。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          Motor_GetCompareValue(motorDutyPercent));
  }
  else if (direction == MOTOR_REVERSE)
  {
    /* 反转：Q3 使用 PA3 PWM，Q2 导通；Q1、Q4 关闭。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); /* Q2 导通。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
                          Motor_GetCompareValue(motorDutyPercent));
  }
  /* 停止时两路 PWM 为 0，PA2 和 PA4 均为高电平。 */

  motorDirection = direction;
}

/* 在当前方向更新 PWM，占空比范围限制为 20%～80%。 */
static void Motor_UpdateDuty(void)
{
  uint32_t compare = Motor_GetCompareValue(motorDutyPercent);

  if (motorDirection == MOTOR_FORWARD)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare);
  }
  else if (motorDirection == MOTOR_REVERSE)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, compare);
  }
}

/* 启动正转和反转 PWM；PA2 已由 MX_GPIO_Init 配置为 GPIO 输出。 */
static void Motor_Init(void)
{
  /* PA1=TIM2_CH2、PA3=TIM2_CH4，两路 PWM 都从 0% 占空比启动。 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);

  /* 先将 PA2 置高，关闭左侧低边 Q2。 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* 上电默认正转，以当前占空比输出 PWM */
  Motor_SetDirection(MOTOR_FORWARD);
}

/* 消抖后只在按键由松开变为按下时返回一次。 */
static uint8_t Button_WasPressed(Button *button)
{
  GPIO_PinState rawState = HAL_GPIO_ReadPin(GPIOB, button->pin);
  uint32_t now = HAL_GetTick();

  if (rawState != button->lastRawState)
  {
    button->lastRawState = rawState;
    button->lastChangeTick = now;
  }

  if ((now - button->lastChangeTick) >= BUTTON_DEBOUNCE_MS &&
      rawState != button->stableState)
  {
    button->stableState = rawState;
    if (button->stableState == GPIO_PIN_RESET)
    {
      return 1U;
    }
  }

  return 0U;
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
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* PB12：正转/反转切换，停止时按下则以正转启动。 */
    if (Button_WasPressed(&buttonDirection))
    {
      Motor_SetDirection((motorDirection == MOTOR_FORWARD) ?
                         MOTOR_REVERSE : MOTOR_FORWARD);
    }
    /* PB13：停止。 */
    else if (Button_WasPressed(&buttonStop))
    {
      Motor_SetDirection(MOTOR_STOP);
    }
    /* PB14：每次加 20%，最高 80% */
    else if (Button_WasPressed(&buttonSpeedUp))
    {
      if (motorDutyPercent < MOTOR_DUTY_MAX)
      {
        motorDutyPercent += MOTOR_DUTY_STEP;
        if (motorDutyPercent > MOTOR_DUTY_MAX)
        {
          motorDutyPercent = MOTOR_DUTY_MAX;
        }
        Motor_UpdateDuty();
      }
    }
    /* PB15：每次减 20%，最低 20%。 */
    else if (Button_WasPressed(&buttonSpeedDown))
    {
      if (motorDutyPercent > MOTOR_DUTY_MIN)
      {
        motorDutyPercent -= MOTOR_DUTY_STEP;
        if (motorDutyPercent < MOTOR_DUTY_MIN)
        {
          motorDutyPercent = MOTOR_DUTY_MIN;
        }
        Motor_UpdateDuty();
      }
    }

    HAL_Delay(5U); /* 按键扫描间隔。 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
