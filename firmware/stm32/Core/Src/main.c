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
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "lsm6ds3.h"
#include "max30102.h"
#include "blockc_data_preprocessing.h"
#include "blockd_motion_metric.h"
#include "blocke_nlms_anc.h"
#include "blockf_hr_estimation.h"
#include "blockg_sqi.h"
#include "blockh_decision_update.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Sample acquisition state-machine
typedef enum
{
	ACQ_IDLE = 0,
	ACQ_READ_PPG,
	ACQ_READ_IMU,
	ACQ_COMMIT
} acq_state_t;

//States for valid & invalid samples in case of failure reading
typedef enum
{
	SAMPLE_VALID = 0,
	SAMPLE_INVALID = 1
} sample_status_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RB_SIZE				         1024U

#define MAX_PENDING_ACQ              2U
#define MAX_CONSEC_FAILS             5U

#define UART_LOG_TIMEOUT_MS          50U
#define RECOVERY_RETRY_INTERVAL_MS   1000U


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---------------- Shared scheduler variables ---------------- */
//GLOBAL VARIABLES for TIM2. It updates once the TIM2 increments
//and sets flag once interrupt occurs.
volatile uint32_t sample_idx = 0;      //global sampple index incremented every timer count
volatile uint32_t acquire_pending = 0; //pending acquire request


/* ---------------- Logical acquisition indexing ---------------- */
static uint32_t started_sample_idx = 0;
static uint32_t acq_sample_idx = 0; //latched index for current acquisiton cycle
static uint32_t commited_sample_idx = 0;

static uint32_t failed_acq_cnt = 0;

//For printing
uint32_t last_written_idx = 0;
uint8_t rb_has_data = 0;

//Acquisition state will first be idle and later be updated through the acquisition layer
static acq_state_t acq_state = ACQ_IDLE;

/* ---------------- Temporary sample staging ---------------- */
//For now, we will keep them as 32 and 16 bits respectively, but later might change.
static uint32_t ppg_ir_raw = 0;
static uint32_t ppg_red_raw = 0;

static int16_t imu_ax_raw = 0;
static int16_t imu_ay_raw = 0;
static int16_t imu_az_raw = 0;

/* ---------------- Ring buffers ---------------- */
uint32_t ppg_ir_rb[RB_SIZE];
static uint32_t ppg_red_rb[RB_SIZE];

int16_t imu_ax_rb[RB_SIZE];
int16_t imu_ay_rb[RB_SIZE];
int16_t imu_az_rb[RB_SIZE];

float imu_mag_rb[RB_SIZE];

uint32_t idx_rb[RB_SIZE];

sample_status_t sample_status_rb[RB_SIZE];

/* ---------------- Recovery ---------------- */
static uint32_t dropped_tick_cnt = 0;
static uint32_t consecutive_fail_cnt = 0;

static uint8_t acquisition_blocked = 0;
static uint32_t last_recovery_attempt_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* ---------------- Function prototypes ---------------- */

static void BlockA_Acquire_Task(void);

static HAL_StatusTypeDef MAX_Read_Sample_Blocking(uint32_t *ir, uint32_t *red);
static HAL_StatusTypeDef LSM_Read_Sample_Blocking(int16_t *ax, int16_t *ay, int16_t *az);

static void Commit_Aligned_Sample(uint32_t idx, uint32_t ir,
		uint32_t red, int16_t ax, int16_t ay, int16_t az);

//Failure Function
static void Commit_Failed_Sample(uint32_t idx);

//Helper wrapper function for UART Logging
static void uart_log (const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), UART_LOG_TIMEOUT_MS);
}

//Function for calculating IMU magnitude
static float imu_mag_approx(int16_t ax, int16_t ay, int16_t az);

//Function for recovering I2C Bus
static void Recover_I2C_Bus(void);

//Function for recovering I2C Bus & Sensors
static HAL_StatusTypeDef Recover_I2C_And_Sensors(void);

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Buffer for UART logging
  char msg[512];


  //For storing the part ID value
  uint8_t part_id = 0;

  uart_log("\r\n Program Begins...\r\n");
  uart_log("Start scanning for PART ID & Initializing Sensors...\r\n");

  //PART ID Scan and Initialization for MAX30102
  if(max30102_read_who_am_i(&hi2c1, &part_id) != HAL_OK){
	  uart_log("MAX30102 Failed Recognizing Part ID\r\n");
	  while(1);
  }

  snprintf(msg, sizeof(msg), "MAX30102 WHO AM I: 0x%02x\r\n", part_id);
  uart_log(msg);

  if(part_id != 0x15){
	  uart_log("MAX30102 Wrong Part ID Recognized.\r\n");
	  while(1);
  }

  if(max30102_init(&hi2c1) != HAL_OK){
	  uart_log("MAX30102 Initialization Failed!\r\n");
	  while(1);
  }

  uart_log("MAX30102 Initialization Complete!\r\n");


  //PART ID Scan and Initialization for LSM6DS3
  if(lsm6ds3_read_who_am_i(&hi2c1, &part_id) != HAL_OK){
	  uart_log("LSM6DS3 Failed Recognizing Part ID\r\n");
	  while(1);
  }

  snprintf(msg, sizeof(msg), "LSM6DS3 WHO AM I: 0x%02x\r\n", part_id);
  uart_log(msg);

  if(part_id != 0x6A){
	  uart_log("LSM6DS3 Wrong Part ID Recognized.\r\n");
	  while(1);
  }

  if(lsm6ds3_init(&hi2c1) != HAL_OK) {
	  uart_log("LSM6DS3 Initialization Failed!\r\n");
	  while(1);
  }


  uart_log("LSM6DS3 Initialization Complete!\r\n");

  //Start the timer.
  HAL_TIM_Base_Start_IT(&htim2);

  uart_log("I2C Scan Complete! \r\n");
  uint32_t last_printed = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  BlockA_Acquire_Task();
	  BlockC_Data_Preprocessing();
	  BlockD_Motion_Metric();
	  BlockE_NLMS_ANC();
	  BlockF_HR_Estimation();
	  BlockG_SQI_Update();
	  BlockH_Decision_Update();

	  if(HAL_GetTick() - last_printed >= 1000) {

		  last_printed = HAL_GetTick();

		  if(rb_has_data) {

			  snprintf(msg, sizeof(msg),
			      "%.1f,%.2f,%lu,%.4f,"
			      "%.1f,%.2f,%.3f,%.2f,%lu,"
			      "%.1f,%.2f,%d,"
			      "%.2f,%.2f,%.2f,%u,%u,%u,"
			      "%.1f\r\n",

			      hr_time_result.hr_bpm,
			      hr_time_result.conf,
			      hr_time_result.n_peaks,
			      hr_time_result.ibi_var,

			      hr_fft_result.hr_bpm,
			      hr_fft_result.conf,
			      hr_fft_result.peak_hz,
			      hr_fft_result.peak_ratio,
			      hr_fft_result.k_peaks,

			      hr_final_result.hr_bpm,
			      hr_final_result.conf,
			      hr_final_result.mode,

			      sqi_result.s_motion,
			      sqi_result.s_time,
			      sqi_result.s_freq,
			      motion_state,
			      sqi_result.time_bad,
			      sqi_result.freq_bad,

			      latest_m
			  );

			  uart_log(msg);

		  }//end if2
	  }//end if1


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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT0_MAX_Pin */
  GPIO_InitStruct.Pin = EXT0_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT0_MAX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT1_MPU_Pin */
  GPIO_InitStruct.Pin = EXT1_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT1_MPU_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------------- Callback function ---------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        sample_idx++;

        if(acquire_pending < MAX_PENDING_ACQ) {

        acquire_pending++;

        }
        else {
        	dropped_tick_cnt++;
        }

    }
}

/* ---------------- Block A for sample acquisition ---------------- */
static void BlockA_Acquire_Task(void) {

	if(acquisition_blocked){

		if(HAL_GetTick() - last_recovery_attempt_ms >= RECOVERY_RETRY_INTERVAL_MS){

			last_recovery_attempt_ms = HAL_GetTick();

			if(Recover_I2C_And_Sensors() == HAL_OK){

				acquisition_blocked = 0;

			}
		}

		return;

	}


	switch(acq_state) {

	case ACQ_IDLE:
	{
		uint8_t start_acq = 0;

		__disable_irq();
		if(acquire_pending > 0) {

			acquire_pending--;
			start_acq = 1;


		}//End if

		__enable_irq();

		if(start_acq){
			//Assign one logical sample index to this entire acquisition cycle
			started_sample_idx++;
			acq_sample_idx = started_sample_idx;

			acq_state = ACQ_READ_PPG;
		}

		break;
	}//End case

	case ACQ_READ_PPG:
	{
		if(MAX_Read_Sample_Blocking(&ppg_ir_raw, &ppg_red_raw) == HAL_OK) {

			acq_state = ACQ_READ_IMU;

		}//End if
		else {

			Commit_Failed_Sample(acq_sample_idx);
			consecutive_fail_cnt++;

			if(consecutive_fail_cnt >= MAX_CONSEC_FAILS){

				if(Recover_I2C_And_Sensors() != HAL_OK) {
					acquisition_blocked = 1;
					last_recovery_attempt_ms = HAL_GetTick();
					acq_state = ACQ_IDLE;
				}
			}
			else {
				acq_state = ACQ_IDLE;
			}

		}//End else
		break;
	}//End case

	case ACQ_READ_IMU:
	{

		if(LSM_Read_Sample_Blocking(&imu_ax_raw, &imu_ay_raw, &imu_az_raw) == HAL_OK) {

			acq_state = ACQ_COMMIT;

		}//End if
		else {

			Commit_Failed_Sample(acq_sample_idx);
			consecutive_fail_cnt++;

			if(consecutive_fail_cnt >= MAX_CONSEC_FAILS){

				if(Recover_I2C_And_Sensors() != HAL_OK) {
					acquisition_blocked = 1;
					last_recovery_attempt_ms = HAL_GetTick();
					acq_state = ACQ_IDLE;
				}
			}
			else {
				acq_state = ACQ_IDLE;
			}

		}//End else
		break;
	}//End case

	case ACQ_COMMIT:
	{

		Commit_Aligned_Sample(acq_sample_idx, ppg_ir_raw, ppg_red_raw,imu_ax_raw,
				imu_ay_raw, imu_az_raw);

		consecutive_fail_cnt = 0;
		acq_state = ACQ_IDLE;

		break;
	}//End case

	default:
	{
		acq_state = ACQ_IDLE;
		break;
	}//End default

	}//End switch

}//End function

/* ---------------- Functions for acquiring samples ---------------- */
static HAL_StatusTypeDef MAX_Read_Sample_Blocking(uint32_t *ir, uint32_t *red){

	max30102_sample_t ppg;
	HAL_StatusTypeDef st;

	if(ir == NULL || red == NULL) {
		return HAL_ERROR;
	}

	st = max30102_read_fifo(&hi2c1, &ppg);
	if(st != HAL_OK){
		return st;
	}

	*red = ppg.red;
	*ir = ppg.ir;

	return HAL_OK;
}
static HAL_StatusTypeDef LSM_Read_Sample_Blocking(int16_t *ax, int16_t *ay, int16_t *az){

	 lsm6ds3_vec3_raw_t acc;
	 HAL_StatusTypeDef st;

	 if(ax == NULL || ay == NULL || az == NULL){
		 return HAL_ERROR;
	 }

	 st = lsm6ds3_read_raw_acc(&hi2c1, &acc);
	 if(st != HAL_OK) {
		 return st;
	 }

	*ax = acc.x;
	*ay = acc.y;
	*az = acc.z;

	return HAL_OK;
}


static void Commit_Aligned_Sample(uint32_t idx, uint32_t ir,
		uint32_t red, int16_t ax, int16_t ay, int16_t az) {

	//For creating a ring buffer index that wraps around at RB_SIZE
	uint32_t w = idx % RB_SIZE;

	//Storing the data to the ring buffers with a corresponding index w in the order of
	//ppg, imu, and idx, respectively.
	ppg_red_rb[w] = red;
	ppg_ir_rb[w] = ir;

	imu_ax_rb[w] = ax;
	imu_ay_rb[w] = ay;
	imu_az_rb[w] = az;

	//Storing IMU MAG into ring buffer
	imu_mag_rb[w] = imu_mag_approx(ax, ay, az);

	idx_rb[w] = idx;

	last_written_idx = idx;
	rb_has_data = 1;

//For storing sample status
	sample_status_rb[w] = SAMPLE_VALID;

	commited_sample_idx++;
}

static void Commit_Failed_Sample(uint32_t idx) {

	uint32_t w = idx % RB_SIZE;

	idx_rb[w] = idx;

	//Stores "0" when data is invalid.
	ppg_red_rb[w] = 0;
	ppg_ir_rb[w] = 0;

	imu_ax_rb[w] = 0;
	imu_ay_rb[w] = 0;
	imu_az_rb[w] = 0;

	imu_mag_rb[w] = 0.0f;

	last_written_idx = idx;
	rb_has_data = 1;

	sample_status_rb[w] = SAMPLE_INVALID;

	failed_acq_cnt ++;
}

/* ---------------- Block B for IMU magnitude approximation ---------------- */

static float imu_mag_approx(int16_t ax, int16_t ay, int16_t az){

	return fabsf(ax) + fabsf(ay) + fabsf(az);

}

/* ---------------- Functions for I2C Recovery during failure ---------------- */

static void Recover_I2C_Bus(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_I2C_DeInit(&hi2c1);

	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(1);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(1);

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);

	for(int i = 0; i < 9; i++){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1);
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);

	GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);

	if(scl == GPIO_PIN_RESET || sda == GPIO_PIN_RESET) {
		uart_log("I2C Bus still low after recovery! \r\n");
	}


	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	MX_I2C1_Init();
}

static HAL_StatusTypeDef Recover_I2C_And_Sensors(void){

	uint8_t part_id = 0;

	char msg[256];

	HAL_TIM_Base_Stop_IT(&htim2);

	Recover_I2C_Bus();

	HAL_Delay(100);

	HAL_StatusTypeDef st;

	st = max30102_read_who_am_i(&hi2c1, &part_id);

	snprintf(msg, sizeof(msg), "MAXREC st:%d, id:0x%02x, err:0x%08lx, state:%d\r\n",
			st, part_id, HAL_I2C_GetError(&hi2c1), HAL_I2C_GetState(&hi2c1));
	uart_log(msg);

	if(st != HAL_OK || part_id != 0x15){
		uart_log("MAX30102 not present during Recovery!\r\n");
		acquire_pending = 0;
		acq_state = ACQ_IDLE;
		consecutive_fail_cnt = 0;
		acquisition_blocked = 1;
		return HAL_ERROR;
	}

	HAL_Delay(200);

	if(max30102_init(&hi2c1) != HAL_OK) {
		uart_log("MAX30102 Reinit failed!\r\n");
		acquire_pending = 0;
		acq_state = ACQ_IDLE;
		consecutive_fail_cnt = 0;
		acquisition_blocked = 1;
		return HAL_ERROR;
	}

	if(lsm6ds3_read_who_am_i(&hi2c1, &part_id) != HAL_OK || part_id != 0x6A){
		uart_log("LSM6DS3 not present during Recovery!\r\n");
		acquire_pending = 0;
		acq_state = ACQ_IDLE;
		consecutive_fail_cnt = 0;
		acquisition_blocked = 1;
		return HAL_ERROR;
	}

	HAL_Delay(200);

	if(lsm6ds3_init(&hi2c1) != HAL_OK) {
		uart_log("LSM6DS3 Reinit failed!\r\n");
		acquire_pending = 0;
		acq_state = ACQ_IDLE;
		consecutive_fail_cnt = 0;
		acquisition_blocked = 1;
		return HAL_ERROR;
	}

	acquire_pending = 0;
	acq_state = ACQ_IDLE;
	consecutive_fail_cnt = 0;

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
		uart_log("TIM2 Restart Failed!r\n");
		return HAL_ERROR;
	}

	uart_log("I2C & Sensors Recovery Executed!\r\n");

	return HAL_OK;
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
