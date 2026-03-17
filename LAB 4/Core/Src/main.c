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
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_psensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  DISPLAY_HUMIDITY = 0,
  DISPLAY_MAGNETO,
  DISPLAY_ACCELERO,
  DISPLAY_PRESSURE,
  DISPLAY_COUNT
} DisplayMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Blue user button B2 on the B-L4S5I-IOT01A board */
#define USER_BUTTON_PORT GPIOC
#define USER_BUTTON_PIN  GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* These variables are shared by the RTOS tasks in Part 2:
 * - sensorTask updates the latest measurements
 * - buttonTask updates which sensor should be displayed
 * - uartTask reads the current selection and prints it */
static char uartBuf[128];

static DisplayMode_t displayMode = DISPLAY_HUMIDITY;

/* One selected quantity from each sensor */
static float humidity_pct = 0.0f;     /* HTS221 */
static int16_t magXYZ[3] = {0};       /* LIS3MDL */
static int16_t accXYZ[3] = {0};       /* LSM6DSL */
static float pressure_hPa = 0.0f;     /* LPS22HB */

static GPIO_PinState lastButtonState = GPIO_PIN_SET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Sensors_Init(void)
{
  if (BSP_HSENSOR_Init() != 0U)
  {
    Error_Handler();
  }

  if (BSP_MAGNETO_Init() != MAGNETO_OK)
  {
    Error_Handler();
  }

  if (BSP_ACCELERO_Init() != ACCELERO_OK)
  {
    Error_Handler();
  }

  if (BSP_PSENSOR_Init() != 0U)
  {
    Error_Handler();
  }
}

void Sensors_ReadAll(void)
{
  /* HTS221: humidity */
  humidity_pct = BSP_HSENSOR_ReadHumidity();

  /* LIS3MDL: magnetometer X/Y/Z */
  BSP_MAGNETO_GetXYZ(magXYZ);

  /* LSM6DSL: accelerometer X/Y/Z */
  BSP_ACCELERO_AccGetXYZ(accXYZ);

  /* LPS22HB: pressure */
  pressure_hPa = BSP_PSENSOR_ReadPressure();
}

void Handle_Button(void)
{
  GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN);

  /* B2 is active-low: detect one falling edge per press */
  if ((lastButtonState == GPIO_PIN_SET) && (currentButtonState == GPIO_PIN_RESET))
  {
    displayMode = (DisplayMode_t)((displayMode + 1) % DISPLAY_COUNT);

    /* In Part 2 this function runs inside buttonTask, so we use osDelay()
     * instead of HAL_Delay() to let the scheduler run the other tasks while
     * the switch signal settles. */
    osDelay(20);
    while (HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET)
    {
      osDelay(5);
    }
  }

  lastButtonState = HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN);
}

void UART_SendSelectedSensor(void)
{
  int len = 0;

  /* The lab handout notes that float formatting can be troublesome when
   * FreeRTOS is enabled, so the scalar sensor values are printed as ints. */
  switch (displayMode)
  {
    case DISPLAY_HUMIDITY:
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "HTS221 Humidity = %d %%RH\r\n",
                     (int)humidity_pct);
      break;

    case DISPLAY_MAGNETO:
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "LIS3MDL Mag XYZ = %d, %d, %d\r\n",
                     magXYZ[0], magXYZ[1], magXYZ[2]);
      break;

    case DISPLAY_ACCELERO:
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "LSM6DSL Acc XYZ = %d, %d, %d\r\n",
                     accXYZ[0], accXYZ[1], accXYZ[2]);
      break;

    case DISPLAY_PRESSURE:
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "LPS22HB Pressure = %d hPa\r\n",
                     (int)pressure_hPa);
      break;

    default:
      len = snprintf(uartBuf, sizeof(uartBuf), "Invalid display mode\r\n");
      break;
  }

  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100);
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  const char *msg = "UART alive\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  Sensors_Init();

   const char *startupMsg =
       "\r\nLab 4 Part 1 started. Press B2 to cycle sensors.\r\n";
   HAL_UART_Transmit(&huart1, (uint8_t *)startupMsg, (uint16_t)strlen(startupMsg), 100);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* After osKernelStart(), application flow is owned by the RTOS scheduler.
     * Reaching this loop would mean the scheduler did not take control. */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
