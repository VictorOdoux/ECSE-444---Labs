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
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  APP_IDLE = 0,
  APP_RECORDING,
  APP_READY,
  APP_PLAYBACK
} app_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX_12B         4095U
#define DAC_MID_12B         2048U

/* Test tone (set to 0 to disable) */
#define TEST_TONE_ENABLE    0
#define TEST_TONE_FS_HZ     16000U
#define TEST_TONE_FREQ_HZ   1000U
#define TEST_TONE_SAMPLES   256U
#define TEST_TONE_CYCLES    ((TEST_TONE_FREQ_HZ * TEST_TONE_SAMPLES) / TEST_TONE_FS_HZ)

/* Recording buffer length:
 * Choose a value that fits in SRAM.
 * Each sample uses:
 *   mic_raw: 4 bytes
 *   dac_buf: 4 bytes (uint32_t for Word DMA)
 * Total bytes ≈ AUDIO_BUF_LEN * 8
 *
 * Example AUDIO_BUF_LEN=16000 -> ~128 kB total.
 */
#define AUDIO_BUF_LEN       16000U

#define LED_BLINK_MS        150U
#define BTN_DEBOUNCE_MS     200U

/* DFSDM data format:
 * 32-bit word where top 24 bits = signed audio, low 8 bits = channel info
 */
#define DFSDM_SHIFT_DISCARD_LSB  8
#define DFSDM_24B_FULL_SCALE     8388608   /* 2^23 */

/* Part 4: note playback settings */
#define NOTE_COUNT           6U
#define NOTE_MS              300U
#define NOTE_GAP_MS          80U
#define SEQ_MAX_NOTE_SAMPLES 5000U
#define SEQ_MAX_GAP_SAMPLES  2000U
#define SEQ_MAX_SAMPLES      (AUDIO_BUF_LEN + (NOTE_COUNT * (SEQ_MAX_NOTE_SAMPLES + SEQ_MAX_GAP_SAMPLES)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Part 3 buffers */
static int32_t  g_mic_raw[AUDIO_BUF_LEN];     /* DFSDM DMA writes 32-bit samples here */
static uint32_t g_dac_buf[AUDIO_BUF_LEN];     /* DAC DMA reads 32-bit words (lower 12 bits used) */

/* App control/state */
static volatile app_state_t g_state = APP_IDLE;
static volatile uint8_t g_btn_event = 0;
static volatile uint8_t g_record_done = 0;

static uint32_t g_last_btn_ms = 0;
static uint32_t g_last_blink_ms = 0;

#if TEST_TONE_ENABLE
static uint32_t g_tone_buf[TEST_TONE_SAMPLES];
#endif

/* Part 4 sequence buffer: notes + recorded sample */
static uint32_t g_seq_buf[SEQ_MAX_SAMPLES];
static uint32_t g_seq_len = 0;
static volatile uint32_t g_seq_idx = 0;
static volatile uint8_t g_seq_active = 0;
static const float g_note_freqs[NOTE_COUNT] = {
  523.25f, /* C5 */
  261.63f, /* C4 */
  659.25f, /* E5 */
  392.00f, /* G4 */
  783.99f, /* G5 */
  329.63f  /* E4 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t TIM2_GetClockHz(void);
static uint32_t DFSDM_GetSampleRateHz(void);
static void TIM2_SetSampleRateHz(uint32_t fs_hz);

#if TEST_TONE_ENABLE
static void GenerateTestTone(void);
static void StartTestTone(void);
#endif

static void StartRecording(void);
static void ProcessMicToDacBuffer(void);
static void StartPlayback(void);
static void StopPlayback(void);
static void BuildSequenceBuffer(uint32_t fs_hz);
static void StartSequencePlayback(uint32_t fs_hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* TIM2 is on APB1. If APB1 prescaler != 1, timer clock = 2*PCLK1. */
static uint32_t TIM2_GetClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

  if (ppre1 < 4U) {
    return pclk1;
  } else {
    return 2U * pclk1;
  }
}

/* DFSDM sampling rate (approx):
 * fs = (SYSCLK / OutputClockDivider) / FOSR
 * You configured:
 *   OutputClock.Selection = SYSTEM
 *   Divider = hdfsdm1_channel2.Init.OutputClock.Divider (e.g., 40)
 *   FOSR = hdfsdm1_filter0.Init.FilterParam.Oversampling (e.g., 40)
 */
static uint32_t DFSDM_GetSampleRateHz(void)
{
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t div    = (uint32_t)hdfsdm1_channel2.Init.OutputClock.Divider;
  uint32_t fosr   = (uint32_t)hdfsdm1_filter0.Init.FilterParam.Oversampling;

  if (div == 0U)  div = 1U;
  if (fosr == 0U) fosr = 1U;

  return (sysclk / (div * fosr));
}

/* Update TIM2 so its update/TRGO frequency matches fs_hz */
static void TIM2_SetSampleRateHz(uint32_t fs_hz)
{
  if (fs_hz == 0U) return;

  uint32_t timclk = TIM2_GetClockHz();
  uint32_t psc    = (uint32_t)htim2.Init.Prescaler + 1U;

  /* ARR = timclk/(psc*fs) - 1 */
  uint64_t denom = (uint64_t)psc * (uint64_t)fs_hz;
  if (denom == 0ULL) return;

  uint64_t arr64 = ((uint64_t)timclk / denom);
  if (arr64 == 0ULL) arr64 = 1ULL;

  uint32_t arr = (uint32_t)arr64 - 1U;

  /* Apply safely */
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_ENABLE(&htim2);
}

#if TEST_TONE_ENABLE
static void GenerateTestTone(void)
{
  /* ~2/3 full-scale to reduce clipping into speaker load */
  const float amp = 1365.0f; /* approx 2/3 of 2047 */
  const float two_pi = 6.283185307f;

  for (uint32_t i = 0; i < TEST_TONE_SAMPLES; i++)
  {
    float phase = two_pi * (float)TEST_TONE_CYCLES * (float)i / (float)TEST_TONE_SAMPLES;
    float s = sinf(phase);
    int32_t y = (int32_t)DAC_MID_12B + (int32_t)(s * amp);
    if (y < 0) y = 0;
    if (y > (int32_t)DAC_MAX_12B) y = (int32_t)DAC_MAX_12B;
    g_tone_buf[i] = (uint32_t)(y & 0x0FFF);
  }
}

static void StartTestTone(void)
{
  TIM2_SetSampleRateHz(TEST_TONE_FS_HZ);

  /* LED solid ON during playback */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  if (HAL_DAC_Start_DMA(&hdac1,
                        DAC_CHANNEL_1,
                        g_tone_buf,
                        TEST_TONE_SAMPLES,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  g_state = APP_PLAYBACK;
}
#endif

static void BuildSequenceBuffer(uint32_t fs_hz)
{
  if (fs_hz == 0U)
  {
    g_seq_len = 0;
    return;
  }

  uint32_t note_samples = (fs_hz * NOTE_MS) / 1000U;
  uint32_t gap_samples = (fs_hz * NOTE_GAP_MS) / 1000U;
  if (note_samples == 0U)
  {
    note_samples = 1U;
  }
  if (gap_samples == 0U)
  {
    gap_samples = 1U;
  }

  uint32_t total_needed = ((note_samples + gap_samples) * NOTE_COUNT) - gap_samples + AUDIO_BUF_LEN;
  if (total_needed > SEQ_MAX_SAMPLES)
  {
    uint32_t max_per_note = (SEQ_MAX_SAMPLES - AUDIO_BUF_LEN) / NOTE_COUNT;
    if (max_per_note == 0U)
    {
      max_per_note = 1U;
    }
    if (max_per_note <= gap_samples)
    {
      gap_samples = max_per_note / 4U;
      if (gap_samples == 0U) gap_samples = 1U;
    }
    note_samples = max_per_note - gap_samples;
    if (note_samples == 0U) note_samples = 1U;
    total_needed = ((note_samples + gap_samples) * NOTE_COUNT) - gap_samples + AUDIO_BUF_LEN;
  }

  const float amp = 1365.0f; /* ~2/3 full-scale to reduce clipping */
  const float two_pi = 6.283185307f;

  uint32_t idx = 0;
  for (uint32_t n = 0; n < NOTE_COUNT; n++)
  {
    float freq = g_note_freqs[n];
    for (uint32_t i = 0; i < note_samples; i++)
    {
      float phase = two_pi * freq * ((float)i / (float)fs_hz);
      float s = sinf(phase);
      int32_t y = (int32_t)DAC_MID_12B + (int32_t)(s * amp);
      if (y < 0) y = 0;
      if (y > (int32_t)DAC_MAX_12B) y = (int32_t)DAC_MAX_12B;
      g_seq_buf[idx++] = (uint32_t)(y & 0x0FFF);
    }
    if (n < (NOTE_COUNT - 1U))
    {
      for (uint32_t i = 0; i < gap_samples; i++)
      {
        g_seq_buf[idx++] = (uint32_t)DAC_MID_12B;
      }
    }
  }

  /* Append recorded sample */
  for (uint32_t i = 0; i < AUDIO_BUF_LEN; i++)
  {
    g_seq_buf[idx++] = g_dac_buf[i] & 0x0FFF;
  }

  g_seq_len = idx;
}

static void StartSequencePlayback(uint32_t fs_hz)
{
  if ((g_seq_len == 0U) || (fs_hz == 0U))
  {
    return;
  }

  g_seq_idx = 0;
  g_seq_active = 1;

  TIM2_SetSampleRateHz(fs_hz);

  /* LED solid ON during playback */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  if (HAL_DAC_Start_DMA(&hdac1,
                        DAC_CHANNEL_1,
                        g_seq_buf,
                        g_seq_len,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  g_state = APP_PLAYBACK;
}

/* Button interrupt: do NOT toggle LED here in Part 3.
 * Just post an event (with debounce), handle in main loop/state machine.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    uint32_t now = HAL_GetTick();
    if ((now - g_last_btn_ms) > BTN_DEBOUNCE_MS)
    {
      g_last_btn_ms = now;
      g_btn_event = 1;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM2) && g_seq_active)
  {
    g_seq_idx++;
    if (g_seq_idx >= g_seq_len)
    {
      g_seq_active = 0;
      (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
      (void)HAL_TIM_Base_Stop_IT(&htim2);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      g_state = APP_IDLE;
    }
  }
}

/* DFSDM: record buffer complete callback */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if (hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    g_record_done = 1;
  }
}

/* Start recording: DFSDM -> DMA -> g_mic_raw[] */
static void StartRecording(void)
{
  /* Stop playback if we were playing */
  StopPlayback();

  g_record_done = 0;
  g_state = APP_RECORDING;

  /* Start blinking LED */
  g_last_blink_ms = HAL_GetTick();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /* Ensure DFSDM DMA is stopped before restarting */
  (void)HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, g_mic_raw, AUDIO_BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Convert DFSDM 32-bit samples -> 12-bit unsigned DAC samples
 * - Discard low 8 bits (channel info)
 * - Signed 24-bit audio remains
 * - Map to 0..4095
 */
static void ProcessMicToDacBuffer(void)
{
  int32_t maxAbs = 0;

  // 1) Find max amplitude in the captured buffer
  for (uint32_t i = 0; i < AUDIO_BUF_LEN; i++)
  {
    int32_t x = (g_mic_raw[i] >> DFSDM_SHIFT_DISCARD_LSB);  // signed
    int32_t a = (x < 0) ? -x : x;
    if (a > maxAbs) maxAbs = a;
  }

  // Avoid divide-by-zero and avoid crazy gain when signal is tiny
  if (maxAbs < 1000) maxAbs = 1000;

  // 2) Normalize to DAC range
  for (uint32_t i = 0; i < AUDIO_BUF_LEN; i++)
  {
    int32_t x = (g_mic_raw[i] >> DFSDM_SHIFT_DISCARD_LSB);

    // scale x to roughly [-2047, +2047]
    int32_t scaled = (int32_t)(((int64_t)x * 2047) / maxAbs);

    int32_t y = (int32_t)DAC_MID_12B + scaled;
    if (y < 0) y = 0;
    if (y > (int32_t)DAC_MAX_12B) y = (int32_t)DAC_MAX_12B;

    g_dac_buf[i] = (uint32_t)(y & 0x0FFF);
  }
}

/* Start playback: DAC DMA paced by TIM2 TRGO */
static void StartPlayback(void)
{
  /* Stop recording DMA (Normal mode would already stop, but safe) */
  (void)HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

  /* Match TIM2 sample rate to DFSDM sample rate */
  uint32_t fs = DFSDM_GetSampleRateHz();
  TIM2_SetSampleRateHz(fs);

  /* LED solid ON during playback */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* Start DAC DMA from recorded audio buffer.
   * IMPORTANT: Your DAC DMA should be configured as CIRCULAR if you want continuous looping
   * until next button press (as the lab requests).
   */
  (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  if (HAL_DAC_Start_DMA(&hdac1,
                        DAC_CHANNEL_1,
                        g_dac_buf,
                        AUDIO_BUF_LEN,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  g_state = APP_PLAYBACK;
}

/* Stop playback: stop DAC DMA and timer, LED off (unless recording blinking) */
static void StopPlayback(void)
{
  if (g_state == APP_PLAYBACK)
  {
    (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    (void)HAL_TIM_Base_Stop_IT(&htim2);
    g_seq_active = 0;

    /* If we're not immediately going to recording, turn LED off */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_state = APP_IDLE;
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  /* Start in IDLE with LED off */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_state = APP_IDLE;

#if TEST_TONE_ENABLE
    GenerateTestTone();
    StartTestTone();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Handle button press event */
	  if (g_btn_event)
	  {
	    g_btn_event = 0;

	    if (g_state == APP_PLAYBACK)
	    {
	      /* Button during playback => stop and start a new recording */
	      StopPlayback();
	      StartRecording();
	    }
	    else if (g_state == APP_READY)
	    {
	      /* Play 6 notes + recorded sample */
	      uint32_t fs = DFSDM_GetSampleRateHz();
	      BuildSequenceBuffer(fs);
	      StartSequencePlayback(fs);
	    }
	    else
	    {
	      /* Button in IDLE or RECORDING => (re)start recording */
	      StartRecording();
	    }
	  }

	  /* Blink LED during recording */
	  if (g_state == APP_RECORDING)
	  {
	    uint32_t now = HAL_GetTick();
	    if ((now - g_last_blink_ms) >= LED_BLINK_MS)
	    {
	      g_last_blink_ms = now;
	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	    }
	  }

	  /* When recording buffer is full: process and wait for play request */
	  if ((g_state == APP_RECORDING) && g_record_done)
	  {
	    g_record_done = 0;

	    /* Stop recording DMA */
	    (void)HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

	    /* Convert mic data -> DAC format */
	    ProcessMicToDacBuffer();

	    /* Ready to play sequence on next button press */
	    g_state = APP_READY;
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
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

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 256;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1813;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
