/*
  Peripheral control library.
  2020-10-04  T. Nakagawa
*/

#include "Peripheral.h"
#include <HardwareTimer.h>
#include <stm32_def.h>
#include <stm32f4xx_hal.h>

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static ADC_HandleTypeDef hadc3;
static DMA_HandleTypeDef hdma_adc1;
static DAC_HandleTypeDef hdac;
static DMA_HandleTypeDef hdma_dac1;
static TIM_HandleTypeDef htim7;
static HardwareTimer *tim;
static void (*adc_callback)();
static void (*tim_callback)();

/*
  System clock configuration.
  HSE=8MHz, HCLK=240MHz, PCLK1=30MHz, PCLK2=60MHz.
*/
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_2);
}

/*
  Initialize ADC1 for 6Msps.
*/
static void MX_ADC1_Init(void) {
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)_Error_Handler(__FILE__, __LINE__);

  multimode.Mode = ADC_TRIPLEMODE_INTERL;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

/*
  Initialize ADC2 for 6Msps.
*/
static void MX_ADC2_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

/*
  Initialize ADC3 for 6Msps.
*/
static void MX_ADC3_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

/*
  Initialize DAC.
*/
static void MX_DAC_Init(void) {
  DAC_ChannelConfTypeDef sConfig;

  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

/*
  Initialize Timer 7 for 46875Hz DAC.
*/
static void MX_TIM7_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1280 - 1;  // 60MHz / 1280 = 46875Hz.
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

/*
  Initialize DMA for DAC and ADC.
*/
static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);	// DAC
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);	// ADC
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/*
  Initialize MCO.
*/
static void MX_MCO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void hal_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  } else if(hadc->Instance==ADC2) {
    __HAL_RCC_ADC2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  } else if(hadc->Instance==ADC3) {
    __HAL_RCC_ADC3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

static void hal_DAC_MspInit(DAC_HandleTypeDef* hdac) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hdac->Instance==DAC) {
    __HAL_RCC_DAC_CLK_ENABLE();
  
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_dac1.Instance = DMA1_Stream5;
    hdma_dac1.Init.Channel = DMA_CHANNEL_7;
    hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dac1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dac1) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(hdac,DMA_Handle1,hdma_dac1);
  }
}

static void hal_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance==TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  adc_callback();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  adc_callback();
}

void initPeripherals() {
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_MCO_Init();
  hal_ADC_MspInit(&hadc1);
  hal_ADC_MspInit(&hadc2);
  hal_ADC_MspInit(&hadc3);
  hal_DAC_MspInit(&hdac);
  hal_TIM_Base_MspInit(&htim7);
}

void startPeripherals(int adc_buf_size, int16_t *adc_buf, void (*adc_handler)(), int dac_buf_size, int16_t *dac_buf) {
  adc_callback = adc_handler;
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t *)adc_buf, adc_buf_size / 2);  // Divided by 2 because sizeof(uint32_t) / sizeof(uint16_t) = 2.

  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dac_buf, dac_buf_size, DAC_ALIGN_12B_R);  
  HAL_TIM_Base_Start(&htim7);
}

void initTimer() {
  tim = new HardwareTimer(TIM14);
  tim->setMode(1, TIMER_OUTPUT_COMPARE, NC);
  tim->setOverflow(1000, HERTZ_FORMAT);	// 1000Hz timer interruption.
}

void startTimer(void (*tim_handler)()) {
  tim_callback = tim_handler;
  tim->attachInterrupt([](HardwareTimer *t) {if (t == tim) tim_callback();});
  tim->setInterruptPriority(3, 0);
  tim->resume();
}
