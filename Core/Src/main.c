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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct IMU_DATA{
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
}IMU_DATA;
typedef struct TCS_COLOUR_DATA {
	uint16_t clear;
	uint16_t red;
	uint16_t green;
	uint16_t blue;
}TCS_COLOUR_DATA;
typedef struct AS_COLOUR_DATA {
	uint16_t ADC_CH_5;
	uint16_t ADC_CH_4;
	uint16_t ADC_CH_3;
	uint16_t ADC_CH_2;
	uint16_t ADC_CH_1;
	uint16_t ADC_CH_0;
}AS_COLOUR_DATA;
typedef enum COMPASS_HEADING{
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
typedef enum STATE_MACHINE{
	navigation,
	found,
	search,
	drop_continue,
	drop_end,
	errorSM
}STATE_MACHINE;
typedef enum DETECTED_COLOUR{
	red,
	blue,
	green,
	brown,
	errorDC
}DETECTED_COLOUR;
typedef struct TCS_COLOUR_CALIBRATION_DATA{
	uint16_t _clear;
	uint16_t _red;
	uint16_t _green;
	uint16_t _blue;
}TCS_COLOUR_CALIBRATION_DATA;
typedef struct AS_COLOUR_CALIBRATION_DATA{
	uint16_t _ADC_CH_5;
	uint16_t _ADC_CH_4;
	uint16_t _ADC_CH_3;
	uint16_t _ADC_CH_2;
	uint16_t _ADC_CH_1;
	uint16_t _ADC_CH_0;
}AS_COLOUR_CALIBRATION_DATA;
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

#define TCS_I2C_DELAY 500

#define TCS_COL_EN_REG 0x80
#define TCS_COL_POWER_NO_WAIT 0x03
#define TCS_COL_POWER_WAIT 0x0B
#define TCS_COL_STATUS_REG 0x93
#define TCS_COL_VALID_BIT 0x01 //bit 0
#define TCS_COL_READ_DATA_REG 0xB4
#define TCS_COL_ATIME_REG 0x81
#define TCS_COL_10_INT_CYCLE 0xF6
#define TCS_COL_42_INT_CYCLE 0xD5

#define AS_I2C_DELAY 500

#define AS_COL_ENABLE_REG 0x80
#define AS_COL_PON_BIT 0x01
#define AS_COL_SP_EN_BIT 0x02
#define AS_COL_CFG0_REG 0xA9
#define AS_COL_REG_BANK_BIT 0x10
#define AS_COL_CFG6_REG 0xAF
#define AS_COL_SMUXEN_BIT 0x10
#define AS_COL_LED_SEL_BIT 0x08
#define AS_COL_CONFIG_REG 0x70
#define AS_COL_LED_REG 0x74
#define AS_COL_LED_ACT_BIT 0x80
#define AS_COL_ATIME_REG 0x81
#define AS_COL_ASTEP_REG_L 0xCA
#define AS_COL_ASTEP_REG_H 0xCB
#define AS_COL_STATUS2_REG 0xA3
#define AS_COL_AVALID_BIT 0x40
#define AS_COL_ASTATUS_REG 0x94

#define TCS_COLOUR_TOLERANCE 500
#define AS_COLOUR_TOLERANCE 25

//----------------------------CONFIG----------------------------------
//colour detection thresholds
#define RClearH 0
#define RClearL 0
#define GClearH 0
#define GClearL 0
#define BClearH 0
#define BClearL 0
#define BrClearH 0
#define BrClearL 0

#define REDH_R_THRESHOLD 17000
#define REDL_R_THRESHOLD 12800
#define REDH_G_THRESHOLD 100000
#define REDL_G_THRESHOLD 0
#define REDH_B_THRESHOLD 100000
#define REDL_B_THRESHOLD 0

#define GREENH_R_THRESHOLD 100000
#define GREENL_R_THRESHOLD 0
#define GREENH_G_THRESHOLD 19500
#define GREENL_G_THRESHOLD 13200
#define GREENH_B_THRESHOLD 100000
#define GREENL_B_THRESHOLD 0

#define BLUEH_R_THRESHOLD 100000
#define BLUEL_R_THRESHOLD 0
#define BLUEH_G_THRESHOLD 100000
#define BLUEL_G_THRESHOLD 0
#define BLUEH_B_THRESHOLD 18000
#define BLUEL_B_THRESHOLD 15000

#define BROWNH_R_THRESHOLD 20000
#define BROWNL_R_THRESHOLD 17000
#define BROWNH_G_THRESHOLD 19500
#define BROWNL_G_THRESHOLD 15000
#define BROWNH_B_THRESHOLD 15500
#define BROWNL_B_THRESHOLD 11200

#define RED_MIN_L 1511
#define RED_MAX_L 45019
#define GREEN_MIN_L 1475
#define GREEN_MAX_L 56166
#define BLUE_MIN_L 1400
#define BLUE_MAX_L 52588

#define RED_MIN_R 1767
#define RED_MAX_R 49031
#define GREEN_MIN_R 1682
#define GREEN_MAX_R 62950
#define BLUE_MIN_R 1625
#define BLUE_MAX_R 59314

#define SERVO_FORWARD 180
#define SERVO_BACKWARD 0
#define SERVO_STOP 90

#define SERVO_180 180
#define SERVO_45 45
#define SERVO_0 0
#define SERVO_90 90
#define SERVO_180 180

//max pulse for forward driving, neutral for stop, min for back
#define SERVO_MAX_PULSE 400
#define SERVO_LEFT_NEUTRAL_PULSE 302
#define SERVO_RIGHT_NEUTRAL_PULSE 303 //you probably forgot to change this
#define SERVO_MIN_PULSE 200

//max pulse for 180, neutral for 90, min for 0
#define SERVOG_MAX_PULSE 400
#define SERVOG_NEUTRAL_PULSE 300
#define SERVOG_MIN_PULSE 200
#define SERVOG_45_PULSE 270

#define SERVOG_10_PULSE 225
#define SERVOG_30_PULSE 250
#define SERVOG_50_PULSE 275
#define SERVOG_70_PULSE 290
#define SERVOG_110_PULSE 320
#define SERVOG_130_PULSE 350
#define SERVOG_150_PULSE 380

#define GRIPPER_DELAY 50
//line following, higher number, sharper turns
#define LINE_TURN_TIME 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LEN];
static const uint8_t ICM20948_ADDR = (0x69 << 1);
static const uint8_t AK09916_ADDR = (0x0C << 1);
static const uint8_t TCS34725_ADDR = (0x29 << 1);
static const uint8_t AS7341_ADDR = (0x39 << 1);
volatile int actualDistance = 0;
volatile STATE_MACHINE state = navigation;
volatile bool legoman_pickedup = false;
volatile bool return_to_start = false;
volatile int left_motor_curr_speed = 0;
volatile int right_motor_curr_speed = 0;
TCS_COLOUR_CALIBRATION_DATA tcs_calibration_data[4];
AS_COLOUR_CALIBRATION_DATA as_calibration_data[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
bool setup_imu_sensor(void);
IMU_DATA read_imu_sensor(void);
int calculateDistanceFromSensor(uint16_t voltage);
COMPASS_HEADING findCardinalDirection(int degrees);
int findCompassHeading(uint16_t x, uint16_t y);
void printCardinalDirection(COMPASS_HEADING direction);
bool setup_tcs_colour_sensor(I2C_HandleTypeDef * hi2c);
TCS_COLOUR_DATA read_tcs_colour_sensor(I2C_HandleTypeDef * hi2c);
bool setup_as_colour_sensor(I2C_HandleTypeDef * hi2c);
AS_COLOUR_DATA read_as_colour_sensor(I2C_HandleTypeDef * hi2c);
void start_motor_pwm(void);
void left_motor_speed(int speed);
void right_motor_speed(int speed);
void gripper_motor_position(int pos);
void follow_line(void);
void check_if_bullseye_crossed(void);
void calibrate_as_colour_sensor(I2C_HandleTypeDef * hi2c, DETECTED_COLOUR colour);
void calibrate_tcs_colour_sensor(I2C_HandleTypeDef * hi2c, DETECTED_COLOUR colour);
void grab_legoman(void);
void spin_back_to_line_following(void);
void check_if_safezone_crossed(void);
void check_if_made_to_end(void);
void stop_and_approach(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile IMU_DATA imu_data;
volatile TCS_COLOUR_DATA tcs_colour_data;
volatile AS_COLOUR_DATA as_colour_data;
uint8_t buf[250];
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  setup_tcs_colour_sensor(&hi2c2); //front left
  setup_tcs_colour_sensor(&hi2c1); //front right
  start_motor_pwm();

  //calibration
//  //Step 1: calibrate front right
//  //place front right over red
//  calibrate_as_colour_sensor(&hi2c1, red);
//  //place front right over green
//  calibrate_as_colour_sensor(&hi2c1, green);
//  //place front right over blue
//  calibrate_as_colour_sensor(&hi2c1, blue);
//  //place front right over brown
//  calibrate_as_colour_sensor(&hi2c1, brown);
//
//  //Step 2: calibrate front left
//  //place front left over red
//  calibrate_as_colour_sensor(&hi2c2, red);
//  //place front left over green
//  calibrate_as_colour_sensor(&hi2c2, green);
//  //place front left over blue
//  calibrate_as_colour_sensor(&hi2c2, blue);
//  //place front left over brown
//  calibrate_as_colour_sensor(&hi2c2, brown);

//  //Step 3: calibrate front right
//  //place back right over red
//  strcpy((char*)buf, "Calibrate Front Right (Red)\n\r");
//  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//  calibrate_tcs_colour_sensor(&hi2c1, red);
//  //place back right over green
////  strcpy((char*)buf, "Calibrate Front Right (Green)\n\r");
////  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
////  calibrate_tcs_colour_sensor(&hi2c1, green);
////  //place back right over blue
////  strcpy((char*)buf, "Calibrate Front Right (Blue)\n\r");
////  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
////  calibrate_tcs_colour_sensor(&hi2c1, blue);
//  strcpy((char*)buf, "Calibrate Front Right (Brown)\n\r");
//  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//  calibrate_tcs_colour_sensor(&hi2c1, brown);
//
//  //Step 4: calibrate front left
//  //place back left over red
//  strcpy((char*)buf, "Calibrate Front Left (Red)\n\r");
//  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//  calibrate_tcs_colour_sensor(&hi2c2, red);
//  //place back left over green
////  strcpy((char*)buf, "Calibrate Front Left (Green)\n\r");
////  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
////  calibrate_tcs_colour_sensor(&hi2c2, green);
////  //place back left over blue
////  strcpy((char*)buf, "Calibrate Front Left (Blue)\n\r");
////   HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
////  calibrate_tcs_colour_sensor(&hi2c2, blue);
//  strcpy((char*)buf, "Calibrate Front Left (Brown)\n\r");
//  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//  calibrate_tcs_colour_sensor(&hi2c2, brown);

  strcpy((char*)buf, "Press Button to start Driving\n\r");
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
	  //wait until button pushed
  }
//
  left_motor_speed(SERVO_FORWARD);
  right_motor_speed(SERVO_FORWARD);
  gripper_motor_position(SERVO_90);

//  spin_back_to_line_following();
//  left_motor_speed(SERVO_FORWARD);
//  right_motor_speed(SERVO_FORWARD);

  //gripper test
//  gripper_motor_position(SERVO_0);
//  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
//	  //wait until button pushed
//  }
//  gripper_motor_position(SERVO_90);
//  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
//	  //wait until button pushed
//  }
//  gripper_motor_position(SERVO_180);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  strcpy((char*)buf, "Press to close Jaw...\n\r");
//	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
//	  	  //wait until button pushed
//	   }
//	  HAL_Delay(2000);
//	  gripper_motor_position(SERVO_0);
//	  strcpy((char*)buf, "Press to 45 degrees Jaw...\n\r");
//	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
//		  //wait until button pushed
//	   }
//	  gripper_motor_position(SERVO_45);
//	  HAL_Delay(2000);
//	  strcpy((char*)buf, "Press to 90 degrees Jaw...\n\r");
//	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
//		  //wait until button pushed
//	   }
//	  gripper_motor_position(SERVO_90);
//	  HAL_Delay(2000);

	  switch (state){
	  case (navigation):
			  follow_line();
	  	  	  check_if_bullseye_crossed();
			  if (!return_to_start && legoman_pickedup){
				  state = found;
			  }
			  break;
	  case (found):
			  stop_and_approach();
	  	  	  grab_legoman();
	  	  	  if (legoman_pickedup && return_to_start){
	  	  		  state = search;
	  	  	  }
			  break;
	  case (search):
			  follow_line();
			  //check_if_safezone_crossed();
			  check_if_made_to_end();
			  if(!legoman_pickedup && return_to_start){
				  state = drop_end;
			  } else if (!legoman_pickedup && !return_to_start) {
				  state = drop_end;
			  }
			  break;
	  case (drop_continue):
			  break;
	  case (drop_end):
			  left_motor_speed(SERVO_STOP);
	  	  	  right_motor_speed(SERVO_STOP);
	  	  	  gripper_motor_position(SERVO_90);
			  break;
	  default:
		  break;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 420;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 420;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
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
  sConfigOC.Pulse = 0;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin_Pin */
  GPIO_InitStruct.Pin = B1_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_Pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int findCompassHeading(uint16_t x, uint16_t y){
	double degrees = 0;
	//x and y are signed, cast to double, multiply by 0.15 to convert to microTesla
	double x_signed = (double)((int16_t)x);
	double y_signed = (double)((int16_t)y);

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
	int actualVoltage = voltage * (3.3/4096);
	//range of sensor is 40cm to 4cm
	//relation is approximately linear between measured voltage and inverse distance
	//equation relating distance to voltage
	//y = mx + b;
	//voltage = 12*(1/distance + 0.42) + b
	//1.25 = 12*(0.09) + b
	//b = 0.17
	//distance = ( 1 / ( (voltage - 0.17) / 12 ) ) - 0.42 ; (cm)
	int distance = ( 1 / ( (actualVoltage - 0.17) / 9.0 ) ) - 0.42;
	return distance;
}
//Called when buffer is completely filled

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
bool setup_tcs_colour_sensor(I2C_HandleTypeDef * hi2c){
    HAL_StatusTypeDef HAL_tcs_ret;
    uint8_t cmd_buf[2];

    //power on sensor
    cmd_buf[0] = TCS_COL_EN_REG;
    cmd_buf[1] = TCS_COL_POWER_NO_WAIT;
    HAL_tcs_ret = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, cmd_buf, 2, TCS_I2C_DELAY);

    //wait at least 2.4ms after first power on
    HAL_Delay(3);

    //increase integration time to 24 ms instead of default 2.4ms
    cmd_buf[0] = TCS_COL_ATIME_REG;
    cmd_buf[1] = TCS_COL_10_INT_CYCLE;
    HAL_tcs_ret = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, cmd_buf, 2, TCS_I2C_DELAY);

    if (HAL_tcs_ret == HAL_OK){
        return true;
    }
    return false;
}
TCS_COLOUR_DATA read_tcs_colour_sensor(I2C_HandleTypeDef * hi2c){
	TCS_COLOUR_DATA data = {0,0,0,0};

	HAL_StatusTypeDef HAL_tcs_ret;
	uint8_t cmd_buf[1];
	uint8_t rec_buf[8];

	uint16_t tcs_read_clear = 0;
	uint16_t tcs_read_red = 0;
	uint16_t tcs_read_green = 0;
	uint16_t tcs_read_blue = 0;

	volatile uint8_t tcs_status = 0;

	//Read status register
	cmd_buf[0] = TCS_COL_STATUS_REG;
	HAL_tcs_ret = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, cmd_buf, 1, TCS_I2C_DELAY);
	HAL_tcs_ret = HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, rec_buf, 1, TCS_I2C_DELAY);
	if(HAL_tcs_ret != HAL_OK)
		return data;
	tcs_status = rec_buf[0];

	//check AVAILD bit for valid output
	while (!(tcs_status & TCS_COL_VALID_BIT)){
		HAL_tcs_ret = HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, rec_buf, 1, TCS_I2C_DELAY);
		tcs_status = rec_buf[0];
	}

	//read colour channel data
	cmd_buf[0] = TCS_COL_READ_DATA_REG;
	HAL_tcs_ret = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, cmd_buf, 1, TCS_I2C_DELAY);
	HAL_tcs_ret = HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, rec_buf, 8, TCS_I2C_DELAY);

	if(HAL_tcs_ret != HAL_OK)
		return data;

	tcs_read_clear = ((uint16_t)rec_buf[1] << 8) | rec_buf[0];
	tcs_read_red = ((uint16_t)rec_buf[3] << 8) | rec_buf[2];
	tcs_read_green = ((uint16_t)rec_buf[5] << 8) | rec_buf[4];
	tcs_read_blue = ((uint16_t)rec_buf[7] << 8) | rec_buf[6];

	data.clear = tcs_read_clear;
	data.red = tcs_read_red;
	data.green = tcs_read_green;
	data.blue = tcs_read_blue;

	return data;
}
bool setup_as_colour_sensor(I2C_HandleTypeDef * hi2c){
	//small delay before attempting to communicate with sensor
	HAL_Delay(200);
	HAL_StatusTypeDef HAL_as_ret;
	uint8_t cmd_buf[20];
	uint8_t rec_buf[20];

	//power on sensor
	uint8_t enable = 0;
	cmd_buf[0] = AS_COL_ENABLE_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	enable = rec_buf[0];
	enable |= AS_COL_PON_BIT;
	cmd_buf[1] = enable;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);

	//access registers 0x60 to 0x74 to enable LED
	//enable reg bank
	uint8_t cfg0 = 0;
	cmd_buf[0] = AS_COL_CFG0_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	cfg0 = rec_buf[0];
	cfg0 |= AS_COL_REG_BANK_BIT;
	cmd_buf[1] = cfg0;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);
	//set LED_SEL in CONFIG
	uint8_t config = 0;
	cmd_buf[0] = AS_COL_CONFIG_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	config = rec_buf[0];
	config |= AS_COL_LED_SEL_BIT;
	cmd_buf[1] = config;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);
	//turn on LED
	uint8_t led = 0;
	cmd_buf[0] = AS_COL_LED_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	led = rec_buf[0];
	led |= AS_COL_LED_ACT_BIT;
	cmd_buf[1] = led;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);

	//re-access 0x80 and up registers
	cmd_buf[0] = AS_COL_CFG0_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	cfg0 = rec_buf[0];
	cfg0 &= ~AS_COL_REG_BANK_BIT;
	cmd_buf[1] = cfg0;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);

	//setup adc timing
	uint8_t atime = 0;
	cmd_buf[0] = AS_COL_ATIME_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	atime = rec_buf[0]; //default is 0
	cmd_buf[1] = 29;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);
	uint8_t asetp = 0;
	cmd_buf[0] = AS_COL_ASTEP_REG_L;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	atime = rec_buf[0]; //default is 999
	//writing 599 but in 2 bytes so byte L is 57 and H is 2
	cmd_buf[1] = 0x57;
	cmd_buf[2] = 0x2;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 3, AS_I2C_DELAY);

	if (HAL_as_ret == HAL_OK){
		return true;
	}
	return false;
}
AS_COLOUR_DATA read_as_colour_sensor(I2C_HandleTypeDef * hi2c){
	AS_COLOUR_DATA data = {0,0,0,0,0,0};

	HAL_StatusTypeDef HAL_as_ret;
	uint8_t cmd_buf[1];
	uint8_t rec_buf[13];

	//start spectral measurement
	uint8_t enable = 0;
	cmd_buf[0] = AS_COL_ENABLE_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
	enable = rec_buf[0];
	enable |= AS_COL_SP_EN_BIT;
	cmd_buf[1] = enable;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 2, AS_I2C_DELAY);

	HAL_Delay(50);

	volatile uint8_t status2 = 0;
	cmd_buf[0] = AS_COL_ENABLE_REG;
	int timeout_count = 10000;
	while (!(status2 & AS_COL_AVALID_BIT) && timeout_count > 0){
		HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
		HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 1, AS_I2C_DELAY);
		status2 = rec_buf[0];
		timeout_count--;
	}

	cmd_buf[0] = AS_COL_ASTATUS_REG;
	HAL_as_ret = HAL_I2C_Master_Transmit(hi2c, AS7341_ADDR, cmd_buf, 1, AS_I2C_DELAY);
	HAL_as_ret = HAL_I2C_Master_Receive(hi2c, AS7341_ADDR, rec_buf, 13, AS_I2C_DELAY);

	uint8_t astatus = rec_buf[0];

	data.ADC_CH_0 = ((uint16_t)rec_buf[2] << 8) | rec_buf[1];
	data.ADC_CH_1 = ((uint16_t)rec_buf[4] << 8) | rec_buf[3];
	data.ADC_CH_2 = ((uint16_t)rec_buf[6] << 8) | rec_buf[5];
	data.ADC_CH_3 = ((uint16_t)rec_buf[8] << 8) | rec_buf[7];
	data.ADC_CH_4 = ((uint16_t)rec_buf[10] << 8) | rec_buf[9];
	data.ADC_CH_5 = ((uint16_t)rec_buf[12] << 8) | rec_buf[11];

	return data;
}
void start_motor_pwm(){
	//motor left
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	//motor right
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
void left_motor_speed(int speed){
	if(speed == SERVO_FORWARD){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MAX_PULSE);
		} else if (speed == SERVO_BACKWARD){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MIN_PULSE);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_LEFT_NEUTRAL_PULSE);
	}
	/*
	//enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	if (speed >= 0){
		__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, speed);	//left forward
		if (speed == 0){
			// disable motor
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	} else {
		__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, abs(speed));	//left backward
	}
	*/
}
void right_motor_speed(int speed){
	if(speed == SERVO_FORWARD){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MIN_PULSE);
	} else if (speed == SERVO_BACKWARD){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_MAX_PULSE);
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_RIGHT_NEUTRAL_PULSE);
	}
	/*
	//enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	if (speed >= 0){
		__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, speed);	//right forward
		if (speed == 0){
			// disable motor
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		}
	} else {
		__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, speed);	//right backward
	}
	*/
}
void gripper_motor_position(int pos){
	if(pos == SERVO_180){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_MAX_PULSE);
	} else if (pos == SERVO_90){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_NEUTRAL_PULSE);
	} else if (pos == SERVO_45){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_45_PULSE);
	}

	else if (pos == 10){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_10_PULSE);
	} else if (pos == 30){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_30_PULSE);
	} else if (pos == 50){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_50_PULSE);
	} else if (pos == 70){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_70_PULSE);
	} else if (pos == 110){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_110_PULSE);
	} else if (pos == 130){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_130_PULSE);
	} else if (pos == 150){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_150_PULSE);
	}

	else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVOG_MIN_PULSE);
	}
}
DETECTED_COLOUR determine_tcs_colour(TCS_COLOUR_DATA data, bool isRight){
	DETECTED_COLOUR colour = errorDC;
	int r = 0;
	int g = 0;
	int b = 0;

	int max = data.red;
	max = max > data.blue ? max : data.blue;
	max = max > data.green ? max : data.green;

	r = (float)data.red / max * 255.0;
	g = (float)data.green / max * 255.0;
	b = (float)data.blue / max * 255.0;

	if (isRight)
	{
		if (r > 250 && g > 200)
		{
			colour = brown;
		}
		else if (r > 250 && g < 150)
		{
			colour = red;
		}
		else if (b > 250 && g < 200)
		{
			colour = blue;
		}
	}
	else
	{
		if (r > 250 && g > 200)
		{
			colour = brown;
			sprintf((char*)buf,
						  "\t brown\n",
						  r, g, b, isRight);
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else if (r > 250 && g < 200)
		{
			colour = red;
			sprintf((char*)buf,
						  "\t RED!\n",
						  r, g, b, isRight);
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else if (b > 250 && g < 200)
		{
			colour = blue;
		}
	}

sprintf((char*)buf,
				  "Red: %d \tGreen: %d \tBlue: %d \tisRight?: %d\n\r\n",
				  r, g, b, isRight);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	/*
	//check for max out of the optical channels
	if(
			data.red > BROWNL_R_THRESHOLD &&
			data.red < BROWNH_R_THRESHOLD &&

			data.green > BROWNL_G_THRESHOLD &&
			data.green < BROWNH_G_THRESHOLD &&

			data.blue > BROWNL_B_THRESHOLD &&
			data.blue < BROWNH_B_THRESHOLD){
		colour = brown;
	} else if (
			data.red > REDL_R_THRESHOLD &&
			data.red < REDH_R_THRESHOLD &&

			data.green > REDL_G_THRESHOLD &&
			data.green < REDH_G_THRESHOLD &&

			data.blue > REDL_B_THRESHOLD &&
			data.blue < REDL_B_THRESHOLD &&

			data.red > data.blue &&
			data.red > data.green){
		colour = red;
	} else if (
			data.red > GREENL_R_THRESHOLD &&
			data.red < GREENH_R_THRESHOLD &&

			data.green > GREENL_G_THRESHOLD &&
			data.green < GREENH_G_THRESHOLD &&

			data.blue > GREENL_B_THRESHOLD &&
			data.blue < GREENL_B_THRESHOLD &&

			data.green > data.red &&
			data.green > data.blue){
		colour = green;
	} else if (
			data.red > BLUEL_R_THRESHOLD &&
			data.red < BLUEH_R_THRESHOLD &&

			data.green > BLUEL_G_THRESHOLD &&
			data.green < BLUEH_G_THRESHOLD &&

			data.blue > BLUEL_B_THRESHOLD &&
			data.blue < BLUEL_B_THRESHOLD &&

			data.blue > data.red &&
			data.blue > data.green){
		colour = blue;
	}
	*/

//	if (data.clear < tcs_calibration_data[red]._clear + TCS_COLOUR_TOLERANCE &&
//		data.clear > tcs_calibration_data[red]._clear - TCS_COLOUR_TOLERANCE &&
//		data.red < tcs_calibration_data[red]._red + TCS_COLOUR_TOLERANCE &&
//		data.red > tcs_calibration_data[red]._red - TCS_COLOUR_TOLERANCE &&
//		data.green < tcs_calibration_data[red]._green + TCS_COLOUR_TOLERANCE &&
//		data.green > tcs_calibration_data[red]._green - TCS_COLOUR_TOLERANCE &&
//		data.blue < tcs_calibration_data[red]._blue + TCS_COLOUR_TOLERANCE &&
//		data.blue > tcs_calibration_data[red]._blue - TCS_COLOUR_TOLERANCE)
//		colour = red;
//	else if (data.clear < tcs_calibration_data[green]._clear + TCS_COLOUR_TOLERANCE &&
//		data.clear > tcs_calibration_data[green]._clear - TCS_COLOUR_TOLERANCE &&
//		data.red < tcs_calibration_data[green]._red + TCS_COLOUR_TOLERANCE &&
//		data.red > tcs_calibration_data[green]._red - TCS_COLOUR_TOLERANCE &&
//		data.green < tcs_calibration_data[green]._green + TCS_COLOUR_TOLERANCE &&
//		data.green > tcs_calibration_data[green]._green - TCS_COLOUR_TOLERANCE &&
//		data.blue < tcs_calibration_data[green]._blue + TCS_COLOUR_TOLERANCE &&
//		data.blue > tcs_calibration_data[green]._blue - TCS_COLOUR_TOLERANCE)
//		colour = green;
//	else if (data.clear < tcs_calibration_data[blue]._clear + TCS_COLOUR_TOLERANCE &&
//		data.clear > tcs_calibration_data[blue]._clear - TCS_COLOUR_TOLERANCE &&
//		data.red < tcs_calibration_data[blue]._red + TCS_COLOUR_TOLERANCE &&
//		data.red > tcs_calibration_data[blue]._red - TCS_COLOUR_TOLERANCE &&
//		data.green < tcs_calibration_data[blue]._green + TCS_COLOUR_TOLERANCE &&
//		data.green > tcs_calibration_data[blue]._green - TCS_COLOUR_TOLERANCE &&
//		data.blue < tcs_calibration_data[blue]._blue + TCS_COLOUR_TOLERANCE &&
//		data.blue > tcs_calibration_data[blue]._blue - TCS_COLOUR_TOLERANCE)
//		colour = blue;
//	else if (data.clear < tcs_calibration_data[brown]._clear + TCS_COLOUR_TOLERANCE &&
//		data.clear > tcs_calibration_data[brown]._clear - TCS_COLOUR_TOLERANCE &&
//		data.red < tcs_calibration_data[brown]._red + TCS_COLOUR_TOLERANCE &&
//		data.red > tcs_calibration_data[brown]._red - TCS_COLOUR_TOLERANCE &&
//		data.green < tcs_calibration_data[brown]._green + TCS_COLOUR_TOLERANCE &&
//		data.green > tcs_calibration_data[brown]._green - TCS_COLOUR_TOLERANCE &&
//		data.blue < tcs_calibration_data[brown]._blue + TCS_COLOUR_TOLERANCE &&
//		data.blue > tcs_calibration_data[brown]._blue - TCS_COLOUR_TOLERANCE)
//		colour = brown;

	return colour;
}
DETECTED_COLOUR determine_as_colour(AS_COLOUR_DATA data){
	DETECTED_COLOUR colour = errorDC;

	if (data.ADC_CH_0 < as_calibration_data[red]._ADC_CH_0 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_0 > as_calibration_data[red]._ADC_CH_0 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 < as_calibration_data[red]._ADC_CH_1 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 > as_calibration_data[red]._ADC_CH_1 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 < as_calibration_data[red]._ADC_CH_2 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 > as_calibration_data[red]._ADC_CH_2 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 < as_calibration_data[red]._ADC_CH_3 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 > as_calibration_data[red]._ADC_CH_3 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 < as_calibration_data[red]._ADC_CH_4 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 > as_calibration_data[red]._ADC_CH_4 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 < as_calibration_data[red]._ADC_CH_5 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 > as_calibration_data[red]._ADC_CH_5 - AS_COLOUR_TOLERANCE)
		colour = red;
	else if (data.ADC_CH_0 < as_calibration_data[green]._ADC_CH_0 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_0 > as_calibration_data[green]._ADC_CH_0 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 < as_calibration_data[green]._ADC_CH_1 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 > as_calibration_data[green]._ADC_CH_1 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 < as_calibration_data[green]._ADC_CH_2 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 > as_calibration_data[green]._ADC_CH_2 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 < as_calibration_data[green]._ADC_CH_3 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 > as_calibration_data[green]._ADC_CH_3 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 < as_calibration_data[green]._ADC_CH_4 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 > as_calibration_data[green]._ADC_CH_4 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 < as_calibration_data[green]._ADC_CH_5 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 > as_calibration_data[green]._ADC_CH_5 - AS_COLOUR_TOLERANCE)
		colour = green;
	else if (data.ADC_CH_0 < as_calibration_data[blue]._ADC_CH_0 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_0 > as_calibration_data[blue]._ADC_CH_0 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 < as_calibration_data[blue]._ADC_CH_1 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 > as_calibration_data[blue]._ADC_CH_1 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 < as_calibration_data[blue]._ADC_CH_2 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 > as_calibration_data[blue]._ADC_CH_2 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 < as_calibration_data[blue]._ADC_CH_3 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 > as_calibration_data[blue]._ADC_CH_3 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 < as_calibration_data[blue]._ADC_CH_4 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 > as_calibration_data[blue]._ADC_CH_4 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 < as_calibration_data[blue]._ADC_CH_5 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 > as_calibration_data[blue]._ADC_CH_5 - AS_COLOUR_TOLERANCE)
		colour = blue;
	else if (data.ADC_CH_0 < as_calibration_data[brown]._ADC_CH_0 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_0 > as_calibration_data[brown]._ADC_CH_0 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 < as_calibration_data[brown]._ADC_CH_1 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_1 > as_calibration_data[brown]._ADC_CH_1 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 < as_calibration_data[brown]._ADC_CH_2 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_2 > as_calibration_data[brown]._ADC_CH_2 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 < as_calibration_data[brown]._ADC_CH_3 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_3 > as_calibration_data[brown]._ADC_CH_3 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 < as_calibration_data[brown]._ADC_CH_4 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_4 > as_calibration_data[brown]._ADC_CH_4 - AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 < as_calibration_data[brown]._ADC_CH_5 + AS_COLOUR_TOLERANCE &&
		data.ADC_CH_5 > as_calibration_data[brown]._ADC_CH_5 - AS_COLOUR_TOLERANCE)
		colour = brown;

	return colour;
}
void follow_line(void){
	//read colour sensor front left
	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
	//read colour sensor from right
	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);

//	sprintf((char*)buf, "LRC: %d | %d\n\rLRRed: %d | %d\n\rLRGre: %d | %d\n\rLRBlu: %d | %d\n\r",
//			left_colour_data.clear, right_colour_data.clear,
//			left_colour_data.red, right_colour_data.red,
//			left_colour_data.green, right_colour_data.green,
//			left_colour_data.blue, right_colour_data.blue);
//	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	sprintf((char*)buf, "Right - C: %d Red: %d Green: %d Blue: %d\n\r", right_colour_data.clear, right_colour_data.red, right_colour_data.green, right_colour_data.blue);
//	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);

	char left_str[100]; //@g change back to 10
	char right_str[100];

	strcpy(left_str, "error");
	strcpy(right_str, "error");


	if (right_colour == red){
		//turn robot slightly right
		right_motor_speed(SERVO_BACKWARD);
		left_motor_speed(SERVO_STOP); //new
		while (right_colour == red)
		{
			right_colour_data = read_tcs_colour_sensor(&hi2c1);
			right_colour = determine_tcs_colour(right_colour_data, true);
		}
//		HAL_Delay(LINE_TURN_TIME);

	} else if (left_colour == red){
		//turn robot slightly left
		left_motor_speed(SERVO_BACKWARD);
		right_motor_speed(SERVO_STOP); //new

		while (left_colour == red)
		{
			left_colour_data = read_tcs_colour_sensor(&hi2c2);
			left_colour = determine_tcs_colour(left_colour_data, false);
		}
//		HAL_Delay(LINE_TURN_TIME);
	}

	right_motor_speed(SERVO_FORWARD);
	left_motor_speed(SERVO_FORWARD); //new

//	if (left_colour == red){
//		//turn robot slightly left
//		left_motor_speed(SERVO_BACKWARD);
//
//		while (left_colour == red)
//		{
//			left_colour_data = read_tcs_colour_sensor(&hi2c2);
//			left_colour = determine_tcs_colour(left_colour_data, false);
//		}
////		HAL_Delay(LINE_TURN_TIME);
//		left_motor_speed(SERVO_FORWARD);
//	} else if (right_colour == red){
//		//turn robot slightly right
//		right_motor_speed(SERV	O_BACKWARD);
//		while (right_colour == red)
//		{
//			right_colour_data = read_tcs_colour_sensor(&hi2c1);
//			right_colour = determine_tcs_colour(right_colour_data, true);
//		}
////		HAL_Delay(LINE_TURN_TIME);
//		right_motor_speed(SERVO_FORWARD);
//	}

	if (left_colour == green){
		strcpy(left_str, "green");
	} else if (left_colour == blue){
		strcpy(left_str, "blue");
	}else if (left_colour == brown){
		strcpy(left_str, "========================brown");
	} else if (left_colour == red){
		strcpy(left_str, "red");
	}
	if (right_colour == green){
		strcpy(right_str, "green");
	}else if (right_colour == blue){
		strcpy(right_str, "blue");
	}else if (right_colour == brown){
		strcpy(right_str, "========================brown");
	} else if (right_colour == red){
		strcpy(right_str, "red");
	}
	sprintf((char*)buf,
				  "Colour: Left =  %s                   Right = %s\n\r",
				  left_str, right_str);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
}
void check_if_bullseye_crossed(void){
	//read colour sensor front left
	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
	//read colour sensor from right
	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);

	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);

	if (left_colour == blue || right_colour == blue){
		legoman_pickedup = true;
		//changes state
	}
}
void calibrate_as_colour_sensor(I2C_HandleTypeDef * hi2c, DETECTED_COLOUR colour){
	//place front right over red then press button (true or 3.3V when not pressed)
	  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
		  //wait until button pushed
	  }
	  as_colour_data = read_as_colour_sensor(hi2c);
	  as_calibration_data[colour]._ADC_CH_0 = as_colour_data.ADC_CH_0;
	  as_calibration_data[colour]._ADC_CH_1 = as_colour_data.ADC_CH_1;
	  as_calibration_data[colour]._ADC_CH_2 = as_colour_data.ADC_CH_2;
	  as_calibration_data[colour]._ADC_CH_3 = as_colour_data.ADC_CH_3;
	  as_calibration_data[colour]._ADC_CH_4 = as_colour_data.ADC_CH_4;
	  as_calibration_data[colour]._ADC_CH_5 = as_colour_data.ADC_CH_5;

	  HAL_Delay(2000);
}
void calibrate_tcs_colour_sensor(I2C_HandleTypeDef * hi2c, DETECTED_COLOUR colour){
	//place front right over red then press button (true or 3.3V when not pressed)
	  while(HAL_GPIO_ReadPin(B1_Pin_GPIO_Port, B1_Pin_Pin)){
		  //wait until button pushed
	  }
	  tcs_colour_data = read_tcs_colour_sensor(hi2c);
	  tcs_calibration_data[colour]._clear = tcs_colour_data.clear;
	  tcs_calibration_data[colour]._red = tcs_colour_data.red;
	  tcs_calibration_data[colour]._green = tcs_colour_data.green;
	  tcs_calibration_data[colour]._blue = tcs_colour_data.blue;

		sprintf((char*)buf, "Clear: %d Red: %d Green: %d Blue: %d\n\r",
				tcs_colour_data.clear,
				tcs_colour_data.red,
				tcs_colour_data.green,
				tcs_colour_data.blue);
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  HAL_Delay(2000);
}
void stop_and_approach(void){
	left_motor_speed(SERVO_STOP);
	right_motor_speed(SERVO_STOP);

	gripper_motor_position(SERVO_90);
	HAL_Delay(200);

	//slow ramp up
	left_motor_speed(SERVO_FORWARD);
	right_motor_speed(SERVO_FORWARD);
//	HAL_Delay(250);
//	left_motor_speed(100);
//	right_motor_speed(100);
//	HAL_Delay(MOTOR_RAMP_DELAY);
//	left_motor_speed(150);
//	right_motor_speed(150);
//	//read colour sensor front left
//	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
//	//read colour sensor from right
//	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);
//
//	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
//	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);
//
//	//until either front sensors detects red
//	while (!(left_colour == red || right_colour == red)){
//		//keep driving forward slowly
//		left_colour_data = read_tcs_colour_sensor(&hi2c2);
//		right_colour_data = read_tcs_colour_sensor(&hi2c1);
//		left_colour = determine_tcs_colour(left_colour_data, false);
//		right_colour = determine_tcs_colour(right_colour_data, true);
//	}
	HAL_Delay(200);
	//stop again
	left_motor_speed(SERVO_STOP);
	right_motor_speed(SERVO_STOP);
}
void grab_legoman(void){
	// OPEN grip
	gripper_motor_position(SERVO_0);
	left_motor_speed(SERVO_STOP);
	right_motor_speed(SERVO_STOP);
	HAL_Delay(2000);

	// Move forward a little bit
//	left_motor_speed(SERVO_FORWARD);
//	right_motor_speed(SERVO_FORWARD);
//	HAL_Delay(100);
//	left_motor_speed(SERVO_STOP);
//	right_motor_speed(SERVO_STOP);
//	HAL_Delay(2000);
	// Close Grip SLOWLY
	gripper_motor_position(10);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(30);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(50);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(70);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(SERVO_90);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(110);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(130);
	HAL_Delay(GRIPPER_DELAY);
	gripper_motor_position(150);
	HAL_Delay(GRIPPER_DELAY);

	gripper_motor_position(SERVO_180);
	HAL_Delay(1000);

//	//drive back
//	left_motor_speed(SERVO_BACKWARD);
//	right_motor_speed(SERVO_BACKWARD);
//	HAL_Delay(1000);
//	left_motor_speed(SERVO_STOP);
//	right_motor_speed(SERVO_STOP);
//	HAL_Delay(1000);

	spin_back_to_line_following();

	legoman_pickedup = true;
	return_to_start = true;
}
void spin_back_to_line_following(void){
	//turn in place until right, then left colour sensors detect red
	left_motor_speed(SERVO_FORWARD);
	right_motor_speed(SERVO_BACKWARD);

	HAL_Delay(2150);

//	//read colour sensor from right
//	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);
//	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);
//
//	while (!(right_colour == red)){
//		right_colour_data = read_tcs_colour_sensor(&hi2c1);
//		right_colour = determine_tcs_colour(right_colour_data, true);
//	}
//
//	while (right_colour == red){
//		right_colour_data = read_tcs_colour_sensor(&hi2c1);
//		right_colour = determine_tcs_colour(right_colour_data, true);
//	}

	//read colour sensor front left
	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
	//until either front sensors detects red
//	while (!(left_colour == red)){
//		//keep driving forward slowly
//		left_colour_data = read_tcs_colour_sensor(&hi2c2);
//		left_colour = determine_tcs_colour(left_colour_data, false);
//	}
	left_motor_speed(SERVO_STOP);
	right_motor_speed(SERVO_STOP);
	HAL_Delay(1000);

	left_motor_speed(SERVO_FORWARD);
	right_motor_speed(SERVO_FORWARD);
	HAL_Delay(750);

	left_motor_speed(SERVO_STOP);
	right_motor_speed(SERVO_STOP);
	HAL_Delay(1000);
}
void check_if_safezone_crossed(void){
	//read colour sensor back left
	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
	//read colour sensor back right
	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);

	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);

	if (left_colour == green || right_colour == green){
		return_to_start = true;
		legoman_pickedup = false;
		//changes state to dropcontinue
	}
}
void check_if_made_to_end(void){
	//read colour sensor left
	TCS_COLOUR_DATA left_colour_data = read_tcs_colour_sensor(&hi2c2);
	//read colour sensor right
	TCS_COLOUR_DATA right_colour_data = read_tcs_colour_sensor(&hi2c1);

	DETECTED_COLOUR left_colour = determine_tcs_colour(left_colour_data, false);
	DETECTED_COLOUR right_colour = determine_tcs_colour(right_colour_data, true);

	if (left_colour == red && right_colour == red){
		return_to_start = false;
		legoman_pickedup = false;
		//changes state to dropend
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
