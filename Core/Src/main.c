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
#include "ssd1306.h"
#include "osc_config.h"
#include "osc_signal.h"
#include "osc_display.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "arm_math.h"

// Filter configuration
#define NUM_TAPS    77
#define BLOCK_SIZE  256    // Process entire ADC buffer at once


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Display and command buffers
uint16_t display_buffer[DISPLAY_SAMPLES];
char cmd_buffer[CMD_BUFFER_SIZE];

// State flags
volatile uint8_t adc_ready = 0;
volatile uint8_t spi_busy = 0;
volatile uint8_t cmd_ready = 0;
volatile uint16_t actual_samples_captured = 0;
volatile uint8_t cmd_index = 0;
uint8_t uart_rx_byte = 0;

// Configuration
OscSettings settings = DEFAULT_SETTINGS;
Measurements measurements = {0};
uint8_t measurements_enabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void apply_settings(OscSettings *s);
static void process_command(char *cmd);
// Call once during initialization:
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Output buffer (Option A — separate from adc_buffer)
static arm_fir_instance_f32 fir_instance;
static float32_t fir_state[NUM_TAPS + BLOCK_SIZE - 1];
static float32_t float_input[BLOCK_SIZE];
static float32_t float_output[BLOCK_SIZE];
static uint16_t filtered_buffer[ADC_BUFFER_SIZE];
static float32_t fir_coeffs[NUM_TAPS] = {
    1.8631301812e-03f,
    -3.5565936674e-03f,
    -5.3983048444e-04f,
    1.1409921345e-03f,
    1.9258013163e-03f,
    1.6321208879e-03f,
    2.2331368232e-04f,
    -1.6885804033e-03f,
    -2.9383877766e-03f,
    -2.4634547529e-03f,
    -1.2418811859e-04f,
    2.9380917437e-03f,
    4.7664795451e-03f,
    3.7705030619e-03f,
    -7.5626596504e-05f,
    -4.8334145470e-03f,
    -7.4760069464e-03f,
    -5.6995282181e-03f,
    3.3884384876e-04f,
    7.5791495209e-03f,
    1.1453169805e-02f,
    8.6056462600e-03f,
    -6.2839764792e-04f,
    -1.1628546414e-02f,
    -1.7545549357e-02f,
    -1.3258668908e-02f,
    9.1439587000e-04f,
    1.8145081823e-02f,
    2.7923187622e-02f,
    2.1720940342e-02f,
    -1.1482108761e-03f,
    -3.1006641974e-02f,
    -5.0622462929e-02f,
    -4.2674169602e-02f,
    1.3011020521e-03f,
    7.5091529166e-02f,
    1.5832874893e-01f,
    2.2383158800e-01f,
    2.4864426289e-01f,
    2.2383158800e-01f,
    1.5832874893e-01f,
    7.5091529166e-02f,
    1.3011020521e-03f,
    -4.2674169602e-02f,
    -5.0622462929e-02f,
    -3.1006641974e-02f,
    -1.1482108761e-03f,
    2.1720940342e-02f,
    2.7923187622e-02f,
    1.8145081823e-02f,
    9.1439587000e-04f,
    -1.3258668908e-02f,
    -1.7545549357e-02f,
    -1.1628546414e-02f,
    -6.2839764792e-04f,
    8.6056462600e-03f,
    1.1453169805e-02f,
    7.5791495209e-03f,
    3.3884384876e-04f,
    -5.6995282181e-03f,
    -7.4760069464e-03f,
    -4.8334145470e-03f,
    -7.5626596504e-05f,
    3.7705030619e-03f,
    4.7664795451e-03f,
    2.9380917437e-03f,
    -1.2418811859e-04f,
    -2.4634547529e-03f,
    -2.9383877766e-03f,
    -1.6885804033e-03f,
    2.2331368232e-04f,
    1.6321208879e-03f,
    1.9258013163e-03f,
    1.1409921345e-03f,
    -5.3983048444e-04f,
    -3.5565936674e-03f,
    1.8631301812e-03f
};

void filter_init(void) {
    arm_fir_init_f32(&fir_instance, NUM_TAPS, fir_coeffs,
                     fir_state, BLOCK_SIZE);
}

void filter_process(uint32_t num_samples) {
    for (uint32_t block = 0; block < num_samples; block += BLOCK_SIZE) {
        uint32_t count = num_samples - block;
        if (count > BLOCK_SIZE) count = BLOCK_SIZE;

        for (uint32_t n = 0; n < count; n++) {
            float_input[n] = (float32_t)adc_buffer[block + n] - 2048.0f;
        }

        arm_fir_f32(&fir_instance, float_input, float_output, count);

        for (uint32_t n = 0; n < count; n++) {
            float32_t temp = float_output[n] + 2048.0f;

            if (temp > 4095.0f)
                temp = 4095.0f;
            else if (temp < 0.0f)
                temp = 0.0f;

            filtered_buffer[block + n] = (uint16_t)temp;
        }
    }
}

/* ==================== HARDWARE CONFIGURATION ==================== */
static void apply_settings(OscSettings *s) {
    // Stop peripherals
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    adc_ready = spi_busy = 0;
    memset(adc_buffer, 0, sizeof(adc_buffer));
    HAL_Delay(2);

    // Calculate sample rate based on mode
    uint64_t window_us = (uint64_t)s->time_div_us * 10;
    uint32_t target_rate, samples_needed;

    if(s->display_mode == DISPLAY_FREQ) {
        target_rate = SR_FFT_MODE;
        samples_needed = FFT_SIZE;
    } else {
        target_rate = window_us ? (ADC_BUFFER_SIZE * 1000000ULL / window_us) : SR_TIME_MODE_MAX;
        if(target_rate > SR_TIME_MODE_MAX) target_rate = SR_TIME_MODE_MAX;
        if(target_rate < 10) target_rate = 10;
        samples_needed = (target_rate * window_us) / 1000000ULL;
        if(samples_needed > ADC_BUFFER_SIZE) samples_needed = ADC_BUFFER_SIZE;
        if(samples_needed < DISPLAY_SAMPLES) samples_needed = DISPLAY_SAMPLES;
    }

    s->sample_rate_hz = target_rate;
    actual_samples_captured = samples_needed;

    // Configure TIM2 (ADC trigger)
    HAL_TIM_Base_DeInit(&htim2);
    uint32_t psc = 0, arr = (SYSTEM_CLOCK_HZ / target_rate) - 1;
    while(arr > 65535) { psc++; arr = (SYSTEM_CLOCK_HZ / (target_rate * (psc + 1))) - 1; }

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = psc;
    htim2.Init.Period = arr;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    TIM_MasterConfigTypeDef master = {.MasterOutputTrigger = TIM_TRGO_UPDATE};
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master);

    // Configure TIM3 (PWM generator)
    uint32_t psc_gen = (s->generator_freq_hz < 1526) ? 99 : 0;
    uint32_t clk_gen = psc_gen ? (TIM3_CLOCK_HZ / 100) : TIM3_CLOCK_HZ;
    uint32_t arr_gen = (clk_gen / s->generator_freq_hz) - 1;
    if(arr_gen > 65535) arr_gen = 65535;
    if(arr_gen < 1) arr_gen = 1;

    htim3.Init.Prescaler = psc_gen;
    htim3.Init.Period = arr_gen;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = (arr_gen * s->duty_cycle_percent) / 100,
        .OCPolarity = TIM_OCPOLARITY_HIGH
    };
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);

    // Start acquisition
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, samples_needed);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim2);
}

/* ==================== COMMAND PROCESSING ==================== */
static void process_command(char *cmd) {
    if(!cmd || !cmd[0]) return;

    int val = (cmd[1] == ':') ? atoi(&cmd[2]) : 0;

    switch(cmd[0]) {
        case 'T':  // Timebase: T:100
            settings.time_div_us = val;
            reset_measurement_filter();
            apply_settings(&settings);
            break;

        case 'F':  // Generator freq: F:1000
            settings.generator_freq_hz = val;
            reset_measurement_filter();
            apply_settings(&settings);
            break;

        case 'D':  // Duty cycle: D:50
            settings.duty_cycle_percent = (val < 1) ? 1 : (val > 99) ? 99 : val;
            reset_measurement_filter();
            apply_settings(&settings);
            break;

        case 'M':  // Mode: M:0/1/2
            if(val <= 2) settings.mode = (ScopeMode)val;
            reset_measurement_filter();
            break;

        case 'X':  // Display mode: X:0/1
            if(val <= 1) {
                settings.display_mode = (DisplayMode)val;
                reset_measurement_filter();
                fft_frame_count = 0;
                apply_settings(&settings);
            }
            break;

        case 'E':  // Measurements: E:0/1
            measurements_enabled = (cmd[2] == '1');
            reset_measurement_filter();
            break;

        case 'R':  // Reset
            if(strcmp(cmd, "RESET") == 0) {
                settings = (OscSettings)DEFAULT_SETTINGS;
                reset_measurement_filter();
                fft_frame_count = 0;
                apply_settings(&settings);
            }
            break;
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
  // FIX: Define variables here so they are visible in the loop
  uint8_t meas_divider = 0;
  uint16_t scratch_buf[DISPLAY_SAMPLES]; // Local buffer for background FFT
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  ssd1306_init();
  ssd1306_clear();
  ssd1306_print(10, 10, "1OSCILLOSCOPE");
  ssd1306_print(20, 25, "INITIALIZING...");
  ssd1306_update();
  HAL_Delay(1000);

  init_fft();
  apply_settings(&settings);
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

  ssd1306_clear();
  ssd1306_print(20, 25, "READY!");
  ssd1306_update();
  HAL_Delay(500);
  /* USER CODE END 2 */
  init_fft();
  filter_init();  // Add this
  /* USER CODE BEGIN WHILE */
  static uint8_t meas_counter = 0;
  static uint8_t oled_counter = 0;
  static uint8_t frames_to_discard = 0;

  while(1) {
      // Process commands
      if(cmd_ready) {
          cmd_ready = 0;
          process_command(cmd_buffer);
          meas_counter = 0;
          frames_to_discard = 3;  // Discard stale frames
      }

      // Process ADC data
      if(adc_ready && !spi_busy) {
          adc_ready = 0;

          if(frames_to_discard > 0) { frames_to_discard--; continue; }
          // In the main loop, inside if(adc_ready):
//          memset(fir_state, 0, sizeof(fir_state));
          filter_process(actual_samples_captured);
          memcpy(adc_buffer, filtered_buffer, actual_samples_captured * sizeof(uint16_t));
          // Measure and prepare display buffer
          if(settings.display_mode == DISPLAY_FREQ) {
              measure_freq_domain(adc_buffer, settings.sample_rate_hz,
                                 display_buffer, DISPLAY_SAMPLES, &measurements);
          } else {
              decimate_samples(adc_buffer, actual_samples_captured,
                              display_buffer, DISPLAY_SAMPLES, settings.mode);
              measure_time_domain(adc_buffer, actual_samples_captured,
                                 settings.sample_rate_hz, &measurements);
          }

          // Send to ESP32 via SPI
          spi_busy = 1;
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
          HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)display_buffer,
                               DISPLAY_SAMPLES * sizeof(uint16_t));

          // Send measurements via UART (every 6 frames)
          if((measurements_enabled || settings.display_mode == DISPLAY_FREQ)
             && ++meas_counter >= 6 && measurements.valid) {
              meas_counter = 0;
              char buf[160];
              snprintf(buf, sizeof(buf),
                  "M:%u,%lu,%u,%u,%u,%lu,%u,%lu,%u,%lu,%u,%lu,%u,%lu,%u\n",
                  measurements.amplitude_mv, measurements.frequency_hz,
                  measurements.period_us, measurements.vrms_mv, measurements.num_peaks,
                  measurements.peak_freqs[0], measurements.peak_mags[0],
                  measurements.peak_freqs[1], measurements.peak_mags[1],
                  measurements.peak_freqs[2], measurements.peak_mags[2],
                  measurements.peak_freqs[3], measurements.peak_mags[3],
                  measurements.peak_freqs[4], measurements.peak_mags[4]);
              HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 20);
          }

          // Update OLED (~12fps)
          if(++oled_counter >= 2) {
              oled_counter = 0;
              ssd1306_clear();

              uint16_t oled_buf[OLED_SAMPLES];
              if(settings.display_mode == DISPLAY_FREQ) {
                  draw_freq_grid();
                  decimate_samples(display_buffer, DISPLAY_SAMPLES,
                                  oled_buf, OLED_SAMPLES, MODE_PEAK_DETECT);
                  draw_spectrum(oled_buf, OLED_SAMPLES);
              } else {
                  draw_grid();
                  decimate_samples(display_buffer, DISPLAY_SAMPLES,
                                  oled_buf, OLED_SAMPLES, MODE_NORMAL);
                  draw_waveform(oled_buf, OLED_SAMPLES);
              }

              // Status line
              char status[32];
              if(settings.display_mode == DISPLAY_FREQ)
                  snprintf(status, 32, "FFT %luHz Pk:%lumV",
                           measurements.frequency_hz, (uint32_t)measurements.amplitude_mv);
              else
                  snprintf(status, 32, "T:%uus F:%luHz",
                           settings.time_div_us, measurements.frequency_hz);
              ssd1306_print(0, 0, status);
              ssd1306_update();
          }
      }

      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, actual_samples_captured);
      HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  // ✅ DISABLE ALL UART INTERRUPTS (critical!)
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);  // Disable RX interrupt
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);    // Disable TX complete
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);   // Disable TX empty
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_PE);    // Disable parity error
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);   // Disable error interrupt
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) adc_ready = 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI2) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        spi_busy = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        if(uart_rx_byte == '\n') {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
            cmd_index = 0;
        } else if(cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = uart_rx_byte;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
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
