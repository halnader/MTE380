/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct IMU_DATA{
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
}IMU_DATA;
typedef enum tag_COMPASS_HEADING{
	N,
	NE,
	E,
	SE,
	S,
	SW,
	W,
	NW,
	ERR,
} COMPASS_HEADING;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 1

#define ICM20948_I2C_DELAY 500

#define ICM20948_INT_PIN_CFG 0x0F

#define ICM20948_MAG_XOUT_L 0x11 //HXL
#define ICM20948_MAG_XOUT_H 0x12 //HXH
#define ICM20948_MAG_YOUT_L 0x13 //HYL
#define ICM20948_MAG_YOUT_H 0x14 //HYH
#define ICM20948_MAG_ZOUT_L 0x15 //HZL
#define ICM20948_MAG_ZOUT_H 0x16 //HZH

#define ICM20948_MAG_STATUS 0x10 //ST1
#define ICM20948_MAG_STATUS2 0x18 //ST2
#define ICM20948_MAG_CNTL2 0x31 //CNTL
#define ICM20948_MAG_CNTL3 0x32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LEN];
static const uint8_t ICM20948_ADDR = (0x68 << 1);
static const uint8_t AK09916_ADDR = (0x0C << 1);
volatile int actualDistance = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
bool setup_imu_sensor(void);
IMU_DATA read_imu_sensor(void);
int calculateDistanceFromSensor(uint16_t voltage);
COMPASS_HEADING findCardinalDirection(int degrees);
int findCompassHeading(uint16_t x, uint16_t y);
void printCardinalDirection(COMPASS_HEADING direction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
IMU_DATA imu_data;
uint8_t buf[48];
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
  HAL_TIM_Base_Start_IT(&htim2);

  setup_imu_sensor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  imu_data = read_imu_sensor();
	  int compassDegrees = findCompassHeading(imu_data.mag_x, imu_data.mag_y);
	  COMPASS_HEADING cardinalDirection = findCardinalDirection(compassDegrees);
	  printCardinalDirection(cardinalDirection);

	  int currentDistance = actualDistance;
	  sprintf((char*)buf, "Current Distance: %d\n\r", currentDistance);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  HAL_Delay(2000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 44;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int findCompassHeading(uint16_t x, uint16_t y){
	double degrees = 0;
	//x and y are signed, cast to double, multiply by 0.15 to convert to microTesla
	double x_signed = (double)x * 0.15;
	double y_signed = (double)y *0.15;

	//equations from honeywell appl. note AN-203
	if (y_signed > 0){
		degrees = 90 - atan(x_signed/y_signed) * 180/M_PI;
	} else if (y_signed < 0) {
		degrees = 270 - atan(x_signed/y_signed) * 180/M_PI;
	} else if (y_signed == 0 && x_signed < 0) {
		degrees = 180;
	} else if (y_signed == 0 && x_signed > 0) {
		degrees = 0;
	} else {
		return -1;
	}

	return (int)degrees;
}
COMPASS_HEADING findCardinalDirection(int degrees){
	if (degrees > 340 && degrees < 20){
		return N;
	}
	else if( degrees > 20 && degrees < 60){
		return NE;
	}
	else if (degrees > 60 && degrees < 110){
		return E;
	}
	else if (degrees > 110 && degrees < 160){
		return SE;
	}
	else if (degrees > 160 && degrees < 200){
		return S;
	}
	else if (degrees > 200 && degrees < 250){
		return SW;
	}
	else if (degrees > 250 && degrees < 290){
		return W;
	}
	else if (degrees > 290 && degrees < 340){
		return NW;
	}
	else{
		return ERR;
	}
}
void printCardinalDirection(COMPASS_HEADING direction){
	switch (direction){
		case N:
			strcpy((char*)buf, "Heading: North\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case NE:
			strcpy((char*)buf, "Heading: North East\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case E:
			strcpy((char*)buf, "Heading: East\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case SE:
			strcpy((char*)buf, "Heading: South East\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case S:
			strcpy((char*)buf, "Heading: South\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case SW:
			strcpy((char*)buf, "Heading: South West\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case W:
			strcpy((char*)buf, "Heading: West\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case NW:
			strcpy((char*)buf, "Heading: North West\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
		case ERR:
			strcpy((char*)buf, "Heading: ERROR\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			break;
	}
}

int calculateDistanceFromSensor(uint16_t voltage){
	//https://learn.sparkfun.com/tutorials/analog-to-digital-conversion/all
	int actualVoltage = voltage * (3.3/65536);
	//range of sensor is 40cm to 4cm
	//relation is approximately linear between measured voltage and inverse distance
	//equation relating distance to voltage
	//y = mx + b;
	//voltage = 12*(1/distance + 0.42) + b
	//1.25 = 12*(0.09) + b
	//b = 0.17
	//distance = ( 1 / ( (voltage - 0.17) / 12 ) ) - 0.42 ; (cm)
	int distance = ( 1 / ( (actualVoltage - 0.17) / 12 ) ) - 0.42;
	return distance;
}
//Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	//ADC reading of analog distance sensor output voltage

	//12-bit adc conversion, using 16bit placeholder
	uint16_t vOut = adc_buf[0];

	actualDistance = calculateDistanceFromSensor(vOut);
}

/**
  * @brief Setup IMU Sensor Function
  * @note	Function used to setup MPU9250.
  * @param None
  * @retval bool Status
  */
bool setup_imu_sensor(void){
	HAL_StatusTypeDef HAL_imu_ret;
	uint8_t cmd_buf[2];
	uint8_t rec_buf[2];

	/*MAGNETOMETER DATA REGISTER READS ---- START*/

	uint8_t mag_id = 0;
	uint8_t mag_control = 0;
	bool mag_setup_status = false;

	uint8_t icm20948_pwr_mgt_2 = 0;
	//pwr mgt 2
	cmd_buf[0] = 0x07;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	icm20948_pwr_mgt_2 = rec_buf[0];

	//turn off gyro and accel
	cmd_buf[0] = 0x07;
	cmd_buf[1] = icm20948_pwr_mgt_2 | 0x3F;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, ICM20948_I2C_DELAY);

	uint8_t icm20948_pwr_mgt_1 = 0;
	//pwr mgt 1
	cmd_buf[0] = 0x06;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	icm20948_pwr_mgt_1 = rec_buf[0];

	//turn on sensor by clearing sleep bit
	cmd_buf[0] = 0x06;
	cmd_buf[1] = icm20948_pwr_mgt_1 & 0xBF;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, ICM20948_I2C_DELAY);

	uint8_t icm20948_int_pin_cfg = 0;
	//Read int_pin_cfg register on icm20948
	cmd_buf[0] = ICM20948_INT_PIN_CFG;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);

	icm20948_int_pin_cfg = rec_buf[0];

	//Set BYPASS bit to high to allow I2C communication from mcu to magnetometer
	cmd_buf[0] = ICM20948_INT_PIN_CFG;
	cmd_buf[1] = icm20948_int_pin_cfg | 0x02;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, ICM20948_I2C_DELAY);

	//Read Device ID, default 0x09
	cmd_buf[0] = 0x01;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_id = rec_buf[0];

	//Read CNTL2 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_control = rec_buf[0];

	//Wait at least 100microseconds before switching to single measurement mode
	HAL_Delay(1);

	//Set power down mode (xxxx0000 == power down mode)
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	cmd_buf[1] = (mag_control & 0x0) & 0xE;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 2, ICM20948_I2C_DELAY);

	if(mag_id == 0x09 && HAL_imu_ret == HAL_OK){
		mag_setup_status = true;
	}

	if(mag_setup_status){
		return true;
	}
	return false;
}
/**
  * @brief Read IMU Sensor Function
  * @note	Function used to read raw gyro, accelerometer, magnetometer values from MPU9250.
  * @param None
  * @retval IMU_DATA Raw sensor data
  */
IMU_DATA read_imu_sensor(void){
	IMU_DATA data;

	HAL_StatusTypeDef HAL_imu_ret;
	uint8_t cmd_buf[2];
	uint8_t rec_buf[2];

	/*MAGNETOMETER DATA REGISTER READS ---- START*/

	int16_t mag_read_x = 0;
	int16_t mag_read_y = 0;
	int16_t mag_read_z = 0;
	volatile uint8_t mag_status = 0;
	uint8_t mag_status2 = 0;
	uint8_t mag_control = 0;

	//Read CNTL1 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_control = rec_buf[0];

	//Set single measurement mode (xxxx0001 == single measurement mode)
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	cmd_buf[1] = (mag_control & 0x0) | 0x1;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 2, ICM20948_I2C_DELAY);

	//Read CNTL1 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_control = rec_buf[0];


	cmd_buf[0] = ICM20948_MAG_STATUS;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_status = rec_buf[0];

	if(mag_status & 0x1){
		//Data ready
		//otherwise data has already been read

		//LittleEndian
		cmd_buf[0] = ICM20948_MAG_XOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, ICM20948_I2C_DELAY);
		mag_read_x = (rec_buf[1] << 8) | rec_buf[0];

		cmd_buf[0] = ICM20948_MAG_YOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, ICM20948_I2C_DELAY);
		mag_read_y = (rec_buf[1] << 8) | rec_buf[0];

		cmd_buf[0] = ICM20948_MAG_ZOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, ICM20948_I2C_DELAY);
		mag_read_z = (rec_buf[1] << 8) | rec_buf[0];
	}

	//Read Status Register 2 (ST2) to signal end of read
	cmd_buf[0] = ICM20948_MAG_STATUS2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, ICM20948_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, ICM20948_I2C_DELAY);
	mag_status2 = rec_buf[0];

	data.mag_x = mag_read_x;
	data.mag_y = mag_read_y;
	data.mag_z = mag_read_z;

	/*MAGNETOMETER DATA REGISTER READS ---- END*/

	return data;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
