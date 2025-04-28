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
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float real;
    float imag;
} complex_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define SIGNAL_SIZE 512
#define NUM_FREQ_BINS 250
#define SAMPLE_RATE 80000.0f
#define DISPLAY_HEIGHT 180

#define NUM_SAMPLES 512   // Total ADC samples for FFT
#define ADC_CYCLES ((NUM_SAMPLES + 2) / 3)  // 43 cycles for 128 samples
volatile uint8_t convCompleted = 0;
uint32_t adcBuf[3 * ADC_CYCLES];  // 3 words per cycle
int fft_adc[NUM_SAMPLES];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static inline complex_t complex_add(complex_t a, complex_t b);
static inline complex_t complex_sub(complex_t a, complex_t b);
static inline complex_t complex_mul(complex_t a, complex_t b);
static inline float complex_abs(complex_t a);
static complex_t twiddle_factor(int k, int N);
void fft_recursive(complex_t* x, complex_t* output, int N);
void map_to_frequency_bins(complex_t* fft_output, float* magnitudes, int fft_size,
                           float sample_rate, float window_mean);
void fft(int* signal, float* display_magnitudes, int signal_size);
void generate_test_signal(int* signal);
void DrawAxis();
void DrawFFTSpectrum(float* display_magnitudes);

void convert_to_float(uint16_t *input, float *output, uint16_t size);

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  //  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValue, 1);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcBuf, 3 * ADC_CYCLES);


  BSP_LCD_Reload(0);
  HAL_Delay(100);
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);  // Layer 0 initialization
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  DrawAxis();

  /*float signal[SIGNAL_SIZE];
  generate_test_signal(signal);
  float display_magnitudes[NUM_FREQ_BINS];
  fft(signal, display_magnitudes, SIGNAL_SIZE);
  DrawFFTSpectrum(display_magnitudes);*/





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
       // Ensure fft_adc array is cleared before each new set of ADC readings
       memset(fft_adc, 0, sizeof(fft_adc));

       for (int i = 0; i < NUM_SAMPLES; i += 3) {
           int adc1 = (int)(adcBuf[i] & 0xFFFF);
           int adc2 = (int)(adcBuf[i + 1] & 0xFFFF);
           int adc3 = (int)(adcBuf[i + 2] & 0xFFFF);

           fft_adc[i] = adc1;
           fft_adc[i+1] = adc2;
           fft_adc[i+2] = adc3;
       }

       // Explicitly clear the magnitudes array before each FFT computation
       float display_magnitudes[NUM_FREQ_BINS] = {0};

       // Perform FFT processing
       float signal[SIGNAL_SIZE];
       generate_test_signal(signal);
       fft(fft_adc, display_magnitudes, SIGNAL_SIZE);

       // Ensure old data is cleared from the display before drawing new data
       BSP_LCD_Clear(LCD_COLOR_WHITE);
       DrawAxis();
       DrawFFTSpectrum(display_magnitudes);

       /* TEMPORARY TEST PRINT
       BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"LCD TEST", CENTER_MODE);
       char fft_value_str[20];

       sprintf(fft_value_str, "FFT[3]: %d", fft_adc[3]);
       BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)fft_value_str, CENTER_MODE);

       sprintf(fft_value_str, "FFT[10]: %d", fft_adc[10]);
       BSP_LCD_DisplayStringAt(0, LINE(9), (uint8_t *)fft_value_str, CENTER_MODE);

       sprintf(fft_value_str, "FFT[25]: %d", fft_adc[25]);
       BSP_LCD_DisplayStringAt(0, LINE(11), (uint8_t *)fft_value_str, CENTER_MODE);*/

       HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_INTERL;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 334;
  hltdc.Init.AccumulatedActiveH = 245;
  hltdc.Init.TotalWidth = 340;
  hltdc.Init.TotalHeigh = 247;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
}


void DrawAxis() {
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  int x_origin = 40;
  int y_origin = 40; // Moved to the top
  int x_end = BSP_LCD_GetXSize() - 20;
  int y_end = BSP_LCD_GetYSize() - 20;

  // Draw new horizontal and vertical axes
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//  BSP_LCD_DrawLine(x_origin, y_origin, x_end, y_origin); // Horizontal line at the top
//  BSP_LCD_DrawLine(x_origin, y_origin, x_origin, y_end); // Vertical axis on the left
  BSP_LCD_DrawHLine(x_origin, y_origin, 180);
  BSP_LCD_DrawVLine(x_origin, y_origin, 250);

  // Add X-axis label (below the x-axis)
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12); // Use a smaller font size
  BSP_LCD_DisplayStringAt(0, 3, (uint8_t *)"Amplitude (dB)", CENTER_MODE); // Y-axis label

  // Add Frequency label, vertically aligned next to the X-axis
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);

  // Calculate the starting position for the label
  int x_pos = x_end + 27;  // Position it next to the X-axis
  int y_pos = 100; // Middle of the Y-axis

  // Iterate through each letter and stack it on top of the previous one
  const char* label = "Frequency (kHz)";
  int offset = -20; // Vertical offset for each character

  // Draw each character one by one vertically (stacked)
  for (int i = 0; label[i] != '\0'; i++) {
    BSP_LCD_DisplayChar(x_pos, y_pos + offset, label[i]);
    offset += 12; // Adjust this value based on font size to stack characters
  }

//  int x_value = 0;
//  for (int i = 0;)

  for (int i = 0; i <= 180; i += 60){
	  BSP_LCD_DrawVLine(x_origin + i, y_origin-5, 5);
  }

  char buffer[10];  // Buffer to hold the string for the number

  for (int i = 0; i <= 180; i += 60) {
      snprintf(buffer, sizeof(buffer), "%d", i / -3);
      BSP_LCD_DisplayStringAt(x_end - i -5, y_origin - 20, (uint8_t *)buffer, LEFT_MODE);
  }


//  for (int i = 0; i <= 222; i += 50){
//	  BSP_LCD_DrawHLine(x_origin - 5, y_origin + i, 5);
//  }
//
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+11, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+22, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+33, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+44, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+55, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+66, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+77, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+88, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+99, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+110, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+121, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+132, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+154, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+165, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+176, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+187, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+198, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+209, 5);
//  BSP_LCD_DrawHLine(x_origin-5, y_origin+222, 5);

  BSP_LCD_DrawHLine(x_origin-5, y_origin+35, 5); //5
  BSP_LCD_DrawHLine(x_origin-5, y_origin+71, 5);  //10
  BSP_LCD_DrawHLine(x_origin-5, y_origin+106, 5); //15
  BSP_LCD_DrawHLine(x_origin-5, y_origin+142, 5); //20
  BSP_LCD_DrawHLine(x_origin-5, y_origin+177, 5); //25
  BSP_LCD_DrawHLine(x_origin-5, y_origin+212, 5); //30
  BSP_LCD_DrawHLine(x_origin-5, y_origin+250, 5); //35




  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+35, (uint8_t *)"5", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+71, (uint8_t *)"10", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+106, (uint8_t *)"15", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+142, (uint8_t *)"20", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+177, (uint8_t *)"25", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+212, (uint8_t *)"30", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_origin - 20, y_origin+250, (uint8_t *)"35", LEFT_MODE);

//  for (int i = 0; i <= 250; i += 50) {
//      snprintf(buffer, sizeof(buffer), "%d", i * 2/25);  // Calculate the value to display (0 to 5)
//      BSP_LCD_DisplayStringAt(x_origin - 20, y_origin + i-5, (uint8_t *)buffer, LEFT_MODE);  // Display the value below the X-axis
//  }



}

void DrawFFTSpectrum(float* display_magnitudes) {
    int x_origin = 40;
    int y_origin = 40;

    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

    // Draw each frequency bin as a vertical line
    for (int i = 0; i < NUM_FREQ_BINS && i < 250; i++) {
        int height = (int)display_magnitudes[i];
        if (height > 0) {
            // Draw a vertical line for each bin
            BSP_LCD_DrawHLine(x_origin, y_origin + i, height);
        }
    }
}

// Complex number operations
static inline complex_t complex_add(complex_t a, complex_t b) {
    complex_t result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

static inline complex_t complex_sub(complex_t a, complex_t b) {


    complex_t result;
    result.real = a.real - b.real;
    result.imag = a.imag - b.imag;
    return result;
}

static inline complex_t complex_mul(complex_t a, complex_t b) {
    complex_t result;
    result.real = a.real * b.real - a.imag * b.imag;
    result.imag = a.real * b.imag + a.imag * b.real;
    return result;
}

static inline float complex_abs(complex_t a) {
    return sqrtf(a.real * a.real + a.imag * a.imag);
}

// Compute complex exponential (e^(-j*2*pi*k/N))
static complex_t twiddle_factor(int k, int N) {
    complex_t result;
    float angle = -2.0f * PI * (float)k / (float)N;
    result.real = cosf(angle);
    result.imag = sinf(angle);
    return result;
}

void fft_recursive(complex_t* x, complex_t* output, int N) {
    // Base case
    if (N == 1) {
        output[0] = x[0];
        return;
    }

    // Allocate memory for even and odd components
    complex_t* x_even = (complex_t*)malloc(N/2 * sizeof(complex_t));
    complex_t* x_odd = (complex_t*)malloc(N/2 * sizeof(complex_t));
    complex_t* X_even = (complex_t*)malloc(N/2 * sizeof(complex_t));
    complex_t* X_odd = (complex_t*)malloc(N/2 * sizeof(complex_t));

    // Split into even and odd indices
    for (int i = 0; i < N/2; i++) {
        x_even[i] = x[2*i];
        x_odd[i] = x[2*i + 1];
    }

    // Recursive FFT on even and odd components
    fft_recursive(x_even, X_even, N/2);
    fft_recursive(x_odd, X_odd, N/2);

    // Combine results with twiddle factors
    for (int k = 0; k < N/2; k++) {
        complex_t tw = twiddle_factor(k, N);
        complex_t p = X_even[k];
        complex_t q = complex_mul(tw, X_odd[k]);

        output[k] = complex_add(p, q);
        output[k + N/2] = complex_sub(p, q);
    }

    // Free allocated memory
    free(x_even);
    free(x_odd);
    free(X_even);
    free(X_odd);
}

//void map_to_frequency_bins(complex_t* fft_output, float* magnitudes, int fft_size,
//                           float sample_rate, float window_mean) {
//    // Calculate frequency resolution of the FFT
//    float freq_resolution = sample_rate / (float)fft_size;
//
//    // Zero out the magnitudes array
//    memset(magnitudes, 0, NUM_FREQ_BINS * sizeof(float));
//
//    // Count how many FFT bins map to each frequency bin
//    int bin_counts[NUM_FREQ_BINS] = {0};
//
//    // Map FFT bins to custom frequency bins
//    for (int i = 0; i < fft_size/2; i++) {
//        // Calculate the frequency this FFT bin represents
//        float bin_freq = i * freq_resolution;
//
//        // Skip frequencies below 20Hz
//        if (bin_freq < 20.0f) {
//            continue;
//        }
//
//        // Calculate which custom bin this frequency falls into
//        // Bin formula: (frequency - 20) / 90
//        int bin_index = (int)((bin_freq - 20.0f) / 90.0f);
//
//        // Ensure we're within the target array bounds
//        if (bin_index >= 0 && bin_index < NUM_FREQ_BINS) {
//            // Accumulate power into appropriate bin
//            float magnitude = complex_abs(fft_output[i]);
//            // Normalize and scale the magnitude
//            magnitude = 2.0f * magnitude / (float)fft_size / window_mean;
//            magnitudes[bin_index] += magnitude;
//            bin_counts[bin_index]++;
//        }
//    }
//
//    // Calculate the average power in each bin and convert back to magnitude
//    for (int i = 0; i < NUM_FREQ_BINS; i++) {
//        if (bin_counts[i] > 0) {
//            // Average the power
//            magnitudes[i] /= bin_counts[i];
//            // Convert power back to magnitude
//            magnitudes[i] = sqrtf(magnitudes[i]);
//        }
//    }
//}

void map_to_frequency_bins(complex_t* fft_output, float* magnitudes, int fft_size,
                           float sample_rate, float window_mean) {
    // Calculate frequency resolution of the FFT
    float freq_resolution = sample_rate / (float)fft_size;

    // Zero out the magnitudes array
    memset(magnitudes, 0, NUM_FREQ_BINS * sizeof(float));

    // Count how many FFT bins map to each frequency bin
    int bin_counts[NUM_FREQ_BINS] = {0};

    // Map FFT bins to custom frequency bins
    for (int i = 0; i < fft_size/2; i++) {
        // Calculate the frequency this FFT bin represents
        float bin_freq = i * freq_resolution;

        // Skip frequencies below 20Hz
        if (bin_freq < 40.0f) {
            continue;
        }

        // Calculate which custom bin this frequency falls into
        int bin_index = (int)(bin_freq / 160.0f);

        // Ensure we're within the target array bounds
        if (bin_index >= 0 && bin_index < NUM_FREQ_BINS) {
            // Accumulate power into appropriate bin
            float magnitude = complex_abs(fft_output[i]);
            // Normalize and scale the magnitude
            magnitude = 2.0f * magnitude / (float)fft_size / window_mean;
            magnitudes[bin_index] += magnitude;
            bin_counts[bin_index]++;
        }
    }

    // Calculate the average power in each bin and convert back to magnitude
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        if (bin_counts[i] > 0) {
            // Average the power
            magnitudes[i] /= bin_counts[i];
            // Convert power back to magnitude
            magnitudes[i] = sqrtf(magnitudes[i]);
        }
    }
}

void fft(int* signal, float* display_magnitudes, int signal_size) {
    // Apply Hanning window - using integers first, then converting to float
    float window_values[SIGNAL_SIZE];
    float window_sum = 0.0f;

    for (int i = 0; i < signal_size; i++) {
        // Hanning window formula: 0.5 * (1 - cos(2π*i/(N-1)))
        window_values[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (signal_size - 1)));
        window_sum += window_values[i];
    }
    float window_mean = window_sum / signal_size;

    // Center the input signal around zero and apply window
    complex_t* complex_signal = (complex_t*)malloc(signal_size * sizeof(complex_t));
    if (!complex_signal) {
        printf("Memory allocation failed\n");
        return;
    }

    for (int i = 0; i < signal_size; i++) {
        float centered_signal = (float)(signal[i] - 2048);
        complex_signal[i].real = centered_signal * window_values[i];
        complex_signal[i].imag = 0.0f;
    }

    // Perform FFT
    complex_t* fft_output = (complex_t*)malloc(signal_size * sizeof(complex_t));
    if (!fft_output) {
        printf("Memory allocation failed\n");
        free(complex_signal);
        return;
    }

    fft_recursive(complex_signal, fft_output, signal_size);

    // Ensure magnitudes array is zeroed out
    float magnitudes[NUM_FREQ_BINS] = {0}; // Clear values before accumulating

    // Map FFT output to frequency bins
    map_to_frequency_bins(fft_output, magnitudes, signal_size, SAMPLE_RATE, window_mean);

    // Find the maximum magnitude for normalization
    float max_magnitude = 0.0f;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        if (magnitudes[i] > max_magnitude) {
            max_magnitude = magnitudes[i];
        }
    }

    // Prevent division by zero
    float scale_factor = (max_magnitude > 0.01f) ? (DISPLAY_HEIGHT / max_magnitude) : (DISPLAY_HEIGHT * 10.0f);

    // Limit scaling factor to prevent extreme values
    if (scale_factor > DISPLAY_HEIGHT * 100.0f) {
        scale_factor = DISPLAY_HEIGHT * 100.0f;
    }

    // Scale magnitudes to fit display
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        float log_scale = 20.0f * log10f(magnitudes[i]);
        display_magnitudes[i] = log_scale * DISPLAY_HEIGHT / 30;

        // Ensure values stay within display bounds
        if (display_magnitudes[i] > DISPLAY_HEIGHT) {
            display_magnitudes[i] = DISPLAY_HEIGHT;
        }
    }

    // Free allocated memory
    free(complex_signal);
    free(fft_output);
}

void generate_test_signal(int* signal) {
    float t[SIGNAL_SIZE];

    // Create time vector
    for (int i = 0; i < SIGNAL_SIZE; i++) {
        t[i] = (float)i / SAMPLE_RATE;
    }

    // Generate a signal with multiple frequencies
    float freq1 = 5000.0f;  // 2 kHz
    float freq2 = 10000.0f;  // 8 kHz
    float freq3 = 15000.0f; // ~20 kHz

    for (int i = 0; i < SIGNAL_SIZE; i++) {
        // Generate floating point signal (-1.0 to 1.0 range)
        float float_signal = 0.1f * sinf(2.0f * PI * freq1 * t[i]) +
                            0.1f * sinf(2.0f * PI * freq2 * t[i]) +
                            0.1f * sinf(2.0f * PI * freq3 * t[i]);

        // Map from [-1.0, 1.0] to [0, 4095] for ADC range
        // First scale to [0, 1.0] then to [0, 4095]
        signal[i] = (int)((float_signal + 1.0f) * 0.5f * 4095.0f);

        // Ensure signal stays within ADC range
        if (signal[i] < 0) signal[i] = 0;
        if (signal[i] > 4095) signal[i] = 4095;
    }
}

void convert_to_float(uint16_t *input, float *output, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        output[i] = (float)input[i];  // Convert each uint16_t to float
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
