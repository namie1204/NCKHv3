/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "ESP32UART.h"
#include "DHT.h"
#include "HLK_LD6002.h"
#include "sms.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_INIT = 0,
  STATE_IDLE,
  STATE_RUN,
  STATE_ERROR,
  STATE_SHUTDOWN,
  STATE_STOP,
  STATE_MAX
} SystemState_t;

typedef enum
{
  EVENT_INIT_SUCCESS = 0,
  EVENT_INIT_FAIL,
  EVENT_START_SUCCESS,
  EVENT_START_FAIL,
  EVENT_ERROR_OCCUR,
  EVENT_SHUTDOWN_REQUEST,
  EVENT_SHUTDOWN_FAIL,
  EVENT_SHUTDOWN_SUCCESS,
  EVENT_RESET,
  EVENT_FORCE_STOP,
  EVENT_MAX
} Event_t;

typedef enum
{
  R_OK = 0,
  R_ERROR,
  R_MAX,
} status_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_RESET_COUNT 3
#define CHECK_INTERVAL 10
#define MAX_PHONE_NUMBERS 1
#define ON 1  // 1: ON, 0: OFF
#define OFF 0 // 1: ON, 0: OFF
#define TRUE 1
#define FALSE 0
#define PRESS 0
#define RELEASE 1
#define DEBUG_BUFFER_SIZE 128
#define DEBUG_UART
#ifdef DEBUG_UART
#define DEBUG_PRINT(fmt, ...)                                                           \
  do                                                                                    \
  {                                                                                     \
    char debug_buf[DEBUG_BUFFER_SIZE];                                                  \
    sprintf(debug_buf, fmt, ##__VA_ARGS__);                                             \
    HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), HAL_MAX_DELAY); \
  } while (0)
#else
#define DEBUG_PRINT(fmt, ...) // Không làm gì nếu DEBUG_UART chưa bật
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
SystemState_t currentState = STATE_INIT;
SystemState_t next_State = STATE_INIT;
Event_t event = EVENT_INIT_FAIL;
RadarData data;
static uint8_t alertSent = 0;
uint32_t startTime = 0; // Luu thoi gian bat dau kiem tra
uint8_t foundChild = 0; // Danh dau phat hien tre

uint8_t reset_count = 0;
uint8_t childDetected = 0;    // 0: ko co tre, 1: phat hien tre
uint8_t vehicleRunning = OFF; // 0: xe dung, 1: xe chay
uint8_t shutdown_request = FALSE;

char *phoneNumbers[] = {"+84833426235"};
uint8_t phoneCount = sizeof(phoneNumbers) / sizeof(phoneNumbers[0]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

status_t Deinit_all(void);
void execute_action(void);
void judge_state(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void execute_action(void)
{
  status_t ret = R_OK;
  switch (currentState)
  {
  case STATE_INIT:
  {
    HLK_LD6002_Init(&huart1);
    ESP32UART_Init(&huart2);
    SMS_Init(&huart3);

    if (ret == R_OK)
    {
      event = EVENT_INIT_SUCCESS;
    }
    else if (ret == R_ERROR)
    {
      event = EVENT_INIT_FAIL;
    }
    break;
  }

  case STATE_IDLE:
  {
    vehicleRunning = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if (vehicleRunning == OFF)
    {
      event = EVENT_START_SUCCESS;
    }
    else
    {
      event = EVENT_START_FAIL;
    }
    break;
  }

  case STATE_RUN:
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == PRESS)
    {
      shutdown_request = TRUE;

      event = EVENT_SHUTDOWN_REQUEST;
    }
    else if (shutdown_request == FALSE)
    {
      HLK_LD6002_ProcessData();
      data = HLK_LD6002_GetRadarData();

      if (data.heart_rate > 0 || data.breath_rate > 0 || data.distance_flag == 1)
      {
        // phat hien tre gui canh bao
        char espData[50];
        snprintf(espData, sizeof(espData), "HR:%d,BR:%d,DIST:%d",
                 (int)data.heart_rate,
                 (int)data.breath_rate,
                 (data.distance_flag) ? (int)data.distance : -1);
        ESP32UART_SendData(espData);
        HAL_UART_Transmit(&huart6, (uint8_t *)espData, strlen(espData), HAL_MAX_DELAY);

        if (!alertSent)
        {
          for (uint8_t i = 0; i < phoneCount; i++)
          {
            SMS_Send(phoneNumbers[i], "Canh bao! Tre bi bo quen tren xe!");
          }
          ESP32UART_SendData("ALERT_CHILD_DETECTED");
          alertSent = 1;
        }
      }
      else
      {
        // ko phat hien tre quet trong 10p
        startTime = HAL_GetTick();
        foundChild = 0;
        while ((HAL_GetTick() - startTime) < (10 * 60 * 1000))
        {
          HLK_LD6002_ProcessData();
          data = HLK_LD6002_GetRadarData();
          if (data.heart_rate > 0 || data.breath_rate > 0 || data.distance_flag == 1)
          {
            foundChild = 1;
            break;
          }
          HAL_Delay(1000);
        }

        if (foundChild)
        {
          for (uint8_t i = 0; i < phoneCount; i++)
          {
            SMS_Send(phoneNumbers[i], "Canh bao! Tre bi bo quen tren xe!");
          }
          ESP32UART_SendData("ALERT_CHILD_DETECTED");
        }
      }
    }

    break;
  }
  case STATE_ERROR:
    /* Dev after handle EEPROM */

    // if (reset_count < MAX_RESET_COUNT)
    // {
    //   reset_count++;
    //   HAL_NVIC_SystemReset(); // Reset
    // }
    // else
    // {
    //   event = EVENT_FORCE_STOP;
    // }

    HAL_NVIC_SystemReset(); // Reset

    break;

  case STATE_SHUTDOWN:
    ret = Deinit_all();

    if (ret == R_OK)
    {
      event = EVENT_SHUTDOWN_SUCCESS;
    }
    else if (ret == R_ERROR)
    {
      event = EVENT_SHUTDOWN_FAIL;
    }
    // ESP32UART_SendData("System shutting down...");
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Tat nguon

    // Standby Mode
    // HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    // HAL_PWR_EnterSTANDBYMode();
    break;
  case STATE_STOP:

    ESP32UART_SendData("System Stopped\r\n");

    // HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    break;
  }
}

void judge_state(void)
{

  switch (currentState)
  {
  case STATE_INIT:
    if (event == EVENT_INIT_SUCCESS)
    {
      DEBUG_PRINT("Init success\r\n");
      next_State = STATE_IDLE;
    }
    else if (event == EVENT_INIT_FAIL)
    {
      DEBUG_PRINT("Init fail\r\n");
      next_State = STATE_ERROR;
    }
    break;

  case STATE_IDLE:
    if (event == EVENT_START_SUCCESS)
    {
      DEBUG_PRINT("Start success\r\n");
      next_State = STATE_RUN;
    }
    else if (event == EVENT_START_FAIL)
    {
      DEBUG_PRINT("Start fail\r\n");
      next_State = STATE_ERROR;
    }
    break;
  case STATE_RUN:
    if (event == EVENT_SHUTDOWN_REQUEST)
    {
      DEBUG_PRINT("Shutdown request\r\n");
      next_State = STATE_SHUTDOWN;
    }
    else if (event == EVENT_ERROR_OCCUR)
    {
      DEBUG_PRINT("Error occur\r\n");
      next_State = STATE_ERROR;
    }
    break;

  case STATE_SHUTDOWN:
    if (event == EVENT_SHUTDOWN_SUCCESS)
    {
      DEBUG_PRINT("Shutdown success\r\n");
      next_State = STATE_STOP;
    }
    else if (event == EVENT_SHUTDOWN_FAIL)
    {
      DEBUG_PRINT("Shutdown fail\r\n");
      next_State = STATE_ERROR;
    }
    break;

  case STATE_ERROR:
    if (event == EVENT_FORCE_STOP)
    {
      DEBUG_PRINT("Force stop\r\n");
      next_State = STATE_STOP;
    }
    else if (event == EVENT_RESET)
    {
      DEBUG_PRINT("Reset\r\n");
      next_State = STATE_INIT;
    }
    break;

  case STATE_STOP:
    /* Do nothing*/
    break;
  }
  currentState = next_State;
}

status_t Deinit_all(void)
{
  status_t ret = R_OK;
  /* TO DO */

  // ESP32UART_Deinit();
  // HLK_LD6002_Deinit();
  // SMS_Deinit();
  return ret;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  DEBUG_PRINT("hhhhh hhhh\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Update state*/
    judge_state();

    switch (currentState)
    {
    case STATE_INIT:

      execute_action();
      break;

    case STATE_IDLE:

      execute_action();
      break;

    case STATE_RUN:

      execute_action();
      break;

    case STATE_ERROR:

      execute_action();
      break;

    case STATE_SHUTDOWN:

      execute_action();
      break;

    case STATE_STOP:
      execute_action();
      return 0;

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1382400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : button_Pin shutdown_Pin */
  GPIO_InitStruct.Pin = button_Pin | shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
