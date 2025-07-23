/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SERVO_interface.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_IN1_PIN	GPIO_PIN_0
#define MOTOR1_IN2_PIN	GPIO_PIN_1

#define MOTOR2_IN3_PIN	GPIO_PIN_2
#define MOTOR2_IN4_PIN	GPIO_PIN_3

#define MOTOR1_GPIO_PORT	GPIOB
#define MOTOR2_GPIO_PORT	GPIOB

#define MOTOR1_PWM	TIM_CHANNEL_1
#define MOTOR2_PWM	TIM_CHANNEL_2
#define SERVO_PWM	TIM_CHANNEL_2

// C
#define TRIG_LEFT1 GPIO_PIN_13  // Left Sensor 1
#define ECHO_LEFT1 GPIO_PIN_14
// A
#define TRIG_LEFT2 GPIO_PIN_5  // Left Sensor 2
#define ECHO_LEFT2 GPIO_PIN_4

// B
#define TRIG_RIGHT1 GPIO_PIN_5  // Right Sensor 1
#define ECHO_RIGHT1 GPIO_PIN_4

//B
#define TRIG_RIGHT2 GPIO_PIN_14  // Right Sensor 2
#define ECHO_RIGHT2 GPIO_PIN_15

// B
#define TRIG_FRONT GPIO_PIN_6
#define ECHO_FRONT GPIO_PIN_7

// B
#define TRIG_BACK GPIO_PIN_13
#define ECHO_BACK GPIO_PIN_12

#define BUZZER_ON()  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
#define BUZZER_OFF() HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)

#define LIGHT_ON()  HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET)
#define LIGHT_OFF() HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET)

#define FAN_ON()  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET)
#define FAN_OFF() HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET)

#define COLLISION_DISTANCE_CM 20
#define COLLISION_CHECK_INTERVAL 200

#define MAX_CMD_LENGTH 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxdata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Movement state variables
uint8_t forward = 0;
uint8_t backward = 0;
uint8_t left = 0;
uint8_t right = 0;
uint8_t rxBuffer[1];
uint8_t self_parking_enabled = 0;

uint32_t last_collision_check = 0;

char AI_rxBuffer[MAX_CMD_LENGTH];
uint8_t rx_index = 0;
uint8_t rx_complete_flag = 0;
uint8_t Parking_Available = 0;
char debug_buffer[100];  // Buffer to send UART

char Dist1[3];
uint8_t Dist1_dash=0;
uint8_t semicolon = ';';

uint16_t temp_read = 0;
// Function prototypes
uint8_t Str_Contains(const char *str, const char *substr);
/*
uint16_t my_strlen(const char *str) {
	uint16_t len = 0;
	while (str[len] != '\0') {
		len++;
	}
	return len;
}
void my_memset(char *arr, char val, uint16_t size) {
	for (uint16_t i = 0; i < size; i++) {
		arr[i] = val;
	}
}
*/
void AI_Handling(char *Massage);
void update_movement();
void set_motor_direction(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, int forward);
void set_motor_speed(uint8_t motor1_speed, uint8_t motor2_speed);


uint32_t get_distance(GPIO_TypeDef* GPIOx, uint16_t TRIG, uint16_t ECHO);
uint8_t check_parking_space();
void self_parking_mode();
void buzzer_beep(uint8_t times, uint16_t delay_ms);

uint8_t check_collision_front();







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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(200);
	// steering motors
	SERVO_voidInit(htim2, TIM_CHANNEL_2);
	// mirror motors
	SERVO_voidInit(htim1, TIM_CHANNEL_1);
	SERVO_voidInit(htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	// Start UART reception in interrupt mode
	HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&AI_rxBuffer[rx_index], 1);

	HAL_TIM_Base_Start(&htim4);
	//uint32_t now;
	//uint16_t temp_mv ;
	//uint16_t temperature;
	/* USER CODE END 2 */



	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/*
		SERVO_voidRotate(TIM_CHANNEL_2, 180);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,250);
		HAL_Delay(1000);
		SERVO_voidRotate(TIM_CHANNEL_2, 90);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,450);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,450);
		HAL_Delay(1000);
		SERVO_voidRotate(TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1050);
		HAL_Delay(1000);*/
		/*
		Dist1_dash = get_distance(GPIOB,TRIG_FRONT, ECHO_FRONT);
		if (Dist1_dash < 0 || Dist1_dash > 99) Dist1_dash = 0;
		Dist1[0]=(Dist1_dash/10+'0');
		Dist1[1]=(Dist1_dash%10+'0');
		HAL_UART_Transmit(&huart2, (uint8_t*)Dist1, sizeof(Dist1), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		 */
		/*
		Dist1_dash = get_distance(GPIOA,TRIG_LEFT2, ECHO_LEFT2);
		if (Dist1_dash < 0 || Dist1_dash > 99) Dist1_dash = 0;
		Dist1[0]=(Dist1_dash/10+'0');
		Dist1[1]=(Dist1_dash%10+'0');
		HAL_UART_Transmit(&huart2, (uint8_t*)Dist1, sizeof(Dist1), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);*/
		/*Dist1_dash = get_distance(GPIOB,TRIG_RIGHT2, ECHO_RIGHT2);
		if (Dist1_dash < 0 || Dist1_dash > 99) Dist1_dash = 0;
		Dist1[0]=(Dist1_dash/10+'0');
		Dist1[1]=(Dist1_dash%10+'0');
		Dist1[2]= 0x0d;
		HAL_UART_Transmit(&huart2, (uint8_t*)Dist1, sizeof(Dist1), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);
		HAL_UART_Transmit(&huart2, &semicolon, sizeof(semicolon), 10);*/
		/*
		now = HAL_GetTick();
		if (now - last_collision_check >= COLLISION_CHECK_INTERVAL) {
			if (check_collision_front()) {
				forward = backward = left = right = 0;
				self_parking_enabled = 0;
			}
			last_collision_check = now;
		}
*/
		if(self_parking_enabled == 1)
		{
			self_parking_mode();
		}
		else
		{
			update_movement();
		}

		/*
		if (rx_complete_flag==1) {
			rx_complete_flag = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*)AI_rxBuffer, my_strlen(AI_rxBuffer), 100);
			// clear after handling
			my_memset(AI_rxBuffer, 0, sizeof(AI_rxBuffer));
		}*/
		/*
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		temp_read = HAL_ADC_GetValue(&hadc1);

		temp_mv = (temp_read * 5000UL) / 4096UL;
		temperature = temp_mv / 10UL;
		// لو عايز تبعتها كسلسلة UART:
		uint8_t temp_int = (uint8_t)temperature;
		Dist1[0] = (temp_int / 10) + '0';
		Dist1[1] = (temp_int % 10) + '0';
		Dist1[2] = 0xd;  // أو 0x0d
		HAL_UART_Transmit(&huart2,(uint8_t*) Dist1, sizeof(Dist1), 10);*/

		//check_parking_space();

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
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 32-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10000-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
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
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
	htim2.Init.Prescaler = 32-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000-1;
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
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 8-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 200-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

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
	huart1.Init.BaudRate = 9600;
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
	huart2.Init.BaudRate = 9600;
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|FAN_Pin|LIGHT_Pin|GPIO_PIN_14
			|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
			|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6
			|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 BUZZER_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_13|BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 FAN_Pin LIGHT_Pin PA14
                           PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|FAN_Pin|LIGHT_Pin|GPIO_PIN_14
			|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB13
                           PB14 PB3 PB5 PB6 
                           PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
			|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6
			|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB15 PB4 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	char command;
	if (huart->Instance == USART1) {
		command = rxBuffer[0];
		// Update movement states based on command
		switch (command) {
		case 'W': forward = 1; backward = 0; break; // Move forward
		case 'S': backward = 1; forward = 0; break; // Move backward
		case 'A': left = 1; break;                 // Turn left
		case 'D': right = 1; break;                // Turn right
		case 'X':                                  // Stop all motion
			forward = 0;
			backward = 0;
			left = 0;
			right = 0;
			self_parking_enabled = 0;
			break;
		case 'P':
			self_parking_enabled = 1;
			break;
		default: break;
		}
		HAL_UART_Receive_IT(&huart1, rxBuffer, 1); // Restart UART reception
	}

	else if (huart->Instance == USART2) {
		char received = AI_rxBuffer[rx_index];
		if (received == '\n' || received == '\r'||received==';') {
			AI_rxBuffer[rx_index] = '\0';  // null terminate
			AI_Handling(AI_rxBuffer);
			rx_complete_flag = 1;
			rx_index = 0;
		} else {
			rx_index++;
			if (rx_index >= MAX_CMD_LENGTH) rx_index = 0;  // avoid overflow
		}
		// Continue receiving next byte
		HAL_UART_Receive_IT(&huart2, (uint8_t*)&AI_rxBuffer[rx_index], 1);


	}
}

void AI_Handling(char *message) {
	if (Str_Contains(message, "light on")) {
		LIGHT_ON();
	}
	else if (Str_Contains(message, "light off")) {
		LIGHT_OFF();
	}
	else if (Str_Contains(message, "fan on")) {
		FAN_ON();
	}
	else if (Str_Contains(message, "fan off")) {
		FAN_OFF();
	}
	else if (Str_Contains(message, "adjust mirror")) {
		SERVO_voidRotate(htim1, TIM_CHANNEL_1, 45);
		SERVO_voidRotate(htim1, TIM_CHANNEL_4, 135);
		HAL_Delay(10);
	}
	else if (Str_Contains(message, "start auto parking")) {
		self_parking_enabled = 1;
	}
	else if (Str_Contains(message, "stop auto parking")) {
		self_parking_enabled = 0;
	}
}

uint8_t Str_Contains(const char *str, const char *substr) {
	while (*str) {
		const char *s = str;
		const char *p = substr;
		while (*s && *p && (*s == *p)) {
			s++;
			p++;
		}
		if (!*p) return 1; // Match found
		str++;
	}
	return 0; // No match
}


void update_movement() {
	uint8_t speed_left = 0;
	uint8_t speed_right = 0;
	uint8_t direction = 1;  // 1 = forward, 0 = backward
	uint8_t angle = 90;

	if (forward || backward) {
		direction = forward ? 1 : 0;

		if (left) {
			speed_left = 60;
			speed_right = 70;
			angle = 45;
		} else if (right) {
			speed_left = 70;
			speed_right = 60;
			angle = 135;
		} else {
			speed_left = speed_right = forward ? 80 : 80;
			angle = 90;
		}
	}
	else if (left || right) {
		speed_left = speed_right = 0;
		angle = left ? 45 : 135;
	}
	else {
		speed_left = speed_right = 0;
		angle = 90;
	}

	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, direction);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, direction);

	set_motor_speed(speed_left, speed_right);

	SERVO_voidRotate(htim2,SERVO_PWM, angle);

	HAL_Delay(10);
}

void set_motor_speed(uint8_t motor1_speed, uint8_t motor2_speed) {

	uint32_t max_speed = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR1_PWM, (motor1_speed * max_speed) / 100);
	__HAL_TIM_SET_COMPARE(&htim3, MOTOR2_PWM, (motor2_speed * max_speed) / 100);
}
void set_motor_direction(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, int forward) {
	if (forward) {
		HAL_GPIO_WritePin(port, pin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(port, pin2, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(port, pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin2, GPIO_PIN_SET);
	}
}

uint32_t get_distance(GPIO_TypeDef* GPIOx, uint16_t TRIG, uint16_t ECHO) {
	uint32_t start_time, end_time,pMillsec,distance;
	HAL_GPIO_WritePin(GPIOx, TRIG, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	while(__HAL_TIM_GET_COUNTER(&htim4)<10);
	HAL_GPIO_WritePin(GPIOx, TRIG, GPIO_PIN_RESET);
	pMillsec = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOx, ECHO)) && pMillsec + 10 >HAL_GetTick());
	start_time = __HAL_TIM_GET_COUNTER(&htim4);

	pMillsec = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	while ((HAL_GPIO_ReadPin (GPIOx, ECHO)) && pMillsec + 50 > HAL_GetTick());
	end_time = __HAL_TIM_GET_COUNTER (&htim4);
	distance = (end_time - start_time)* 0.034/2;
	HAL_Delay(10);
	return distance;
}
// Updated Self-Parking Algorithm Inspired by Original Design
/*void self_parking_mode() {  /////
	uint32_t dist_rf;
	uint32_t dist_rb;
	uint32_t dist_front = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);
	uint32_t dist_back = get_distance(GPIOB, TRIG_BACK, ECHO_BACK);
	dist_rb = get_distance(GPIOB, TRIG_RIGHT2, ECHO_RIGHT2);
	dist_rf = get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1);
	SERVO_voidRotate(htim2, SERVO_PWM, 90);
    HAL_Delay(100);
	if((Distance_F <= 20)){
		set_motor_speed(0, 0);
	}
	// first block

	if((dist_rb <= 20) && (dist_rf <= 20) && (Parking_Available == 0))
	{
		// Start moving forward slowly
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(70, 70);
	}
	//finding position
	else if((Distance_Rf > 20) && (Distance_Rb > 20) && (Parking_Available == 0))
	{
		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		Parking_Available = 1;
	}


}*/

void self_parking_mode() {
	/*uint8_t right_space_counter = 0;
    uint8_t space_found = 0;
    uint32_t start_time = HAL_GetTick();
    uint32_t search_time_limit = 5000; // Search up to 10 seconds
    uint32_t dist_rf;
    uint32_t dist_rb;
    uint32_t dist_front;
    BUZZER_ON();  // Start buzzer to indicate search
    //buzzer_beep(2, 100);
    SERVO_voidRotate(htim2, SERVO_PWM, 90);
    HAL_Delay(100);

    // Start moving forward slowly
    set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
    set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
    set_motor_speed(40, 40);*/
	uint32_t dist_rf;
	uint32_t dist_rb;
	uint32_t dist_f;
	uint32_t dist_b;
	uint8_t space_found = 0;
	uint32_t space_start_time = 0;
	uint32_t search_start_time = HAL_GetTick();
	uint8_t space_detected = 0;

	BUZZER_ON();
	SERVO_voidRotate(htim2, SERVO_PWM, 90);
	HAL_Delay(100);

	// Start moving forward slowly
	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
	set_motor_speed(60, 60);

	while (HAL_GetTick() - search_start_time < 10000) {
		dist_rf = get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1);
		dist_rb = get_distance(GPIOB, TRIG_RIGHT2, ECHO_RIGHT2);
		dist_b  = get_distance(GPIOB, TRIG_BACK, ECHO_BACK);
		dist_f  = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);

		// 🟡 Debug info on UART
		sprintf(debug_buffer, "Searching... RF: %lu RB: %lu F: %lu B: %lu\r\n", dist_rf, dist_rb, dist_f, dist_b);
		HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);

		if (dist_rf > 30 && dist_rb > 30) {
			if (!space_detected) {
				space_start_time = HAL_GetTick();
				space_detected = 1;
			} else {
				uint32_t duration = HAL_GetTick() - space_start_time;
				if (duration >= 1500) {
					space_found = 1;
					set_motor_speed(0, 0);
					HAL_Delay(100);
					sprintf(debug_buffer, "✅ Space Found. Starting Parking\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
					break;
				}
			}
		} else {
			space_detected = 0;
		}

		HAL_Delay(20);
	}

	if (!space_found || !self_parking_enabled) {
		BUZZER_OFF();
		self_parking_enabled = 0;
		sprintf(debug_buffer, "❌ No Space Found\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
		return;
	}

	if (space_found) {
		buzzer_beep(1, 200);
	}

	//---------
	set_motor_speed(0, 0);  // Stop after finding space or timeout
	HAL_Delay(200);

	// ===== Begin parking maneuver =====

	// Step 1: Go backward while steering right
	SERVO_voidRotate(htim2, SERVO_PWM, 135);
	HAL_Delay(100);

	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
	set_motor_speed(70, 60);
	HAL_Delay(500);
	//dist_rb = get_distance(GPIOB, TRIG_RIGHT2, ECHO_RIGHT2);
	uint32_t dist_back = get_distance(GPIOB, TRIG_BACK, ECHO_BACK);
	while (dist_back > 15 && self_parking_enabled) {
		dist_back = get_distance(GPIOB, TRIG_BACK, ECHO_BACK);
		sprintf(debug_buffer, "STEP 1: Backing... B: %lu\r\n", dist_back);
		HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
		HAL_Delay(50);
	}

	HAL_Delay(800);
	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
	set_motor_speed(60,60);
	HAL_Delay(200);

	set_motor_speed(0, 0);
	HAL_Delay(100);
	// Step 2: Go forward while steering left to align
	SERVO_voidRotate(htim2, SERVO_PWM,45);
	HAL_Delay(100);

	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
	set_motor_speed(60,70);
	HAL_Delay(500);
	dist_rf = get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1);
	dist_f = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);

	while ((dist_rf > 20 || dist_f > 18) && self_parking_enabled) {
		dist_rf = get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1);
		dist_f = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);
		HAL_Delay(50);
	}
	set_motor_speed(0, 0);
	SERVO_voidRotate(htim2, SERVO_PWM, 90);
	HAL_Delay(100);
	self_parking_enabled = 0;
	BUZZER_OFF();
	buzzer_beep(2, 200);  // Indicate success

}


/*
void self_parking_mode() {
	uint8_t space_found = 0;
	uint32_t start_time = HAL_GetTick();
	uint32_t search_time_limit = 5000; // Search for up to 10 seconds
	// Start buzzer to indicate parking
	BUZZER_ON();
	// Move forward to search for a parking space
	SERVO_voidRotate(htim2,SERVO_PWM, 90);
	HAL_Delay(100);

	set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
	set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
	set_motor_speed(50, 50);

	// Search loop
	while (!space_found) {
		space_found = check_parking_space();

		if (HAL_GetTick() - start_time > search_time_limit) {
			set_motor_speed(0, 0);
			self_parking_enabled = 0;

            //char msg[] = "Parking Failed: No Space Found\r\n";
            //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
			return;
		}

		HAL_Delay(100);
	}

	// Stop car when space is found
	set_motor_speed(0, 0);
	if (!self_parking_enabled) {
		BUZZER_OFF();
		return; // 🚫 Exit if stopped externally
	}
	HAL_Delay(200);

	// Perform parking maneuver
	if (space_found == 1 && self_parking_enabled) {  // Left-side parking
		// Reverse while steering left
		SERVO_voidRotate(htim2,SERVO_PWM, 45);
		HAL_Delay(100);

		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
		set_motor_speed(40, 60);

		// Stop when back sensor reads < 15 cm
		while (get_distance(GPIOB, TRIG_BACK, ECHO_BACK) > 15  && self_parking_enabled) {
			HAL_Delay(50);
		}

		if (!self_parking_enabled) { set_motor_speed(0,0); BUZZER_OFF(); return; }
		set_motor_speed(0,0);
		// Move forward and steer right to align
		SERVO_voidRotate(htim2,SERVO_PWM, 135);
		HAL_Delay(100);

		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(50, 50);

		while (get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT) > 20 && self_parking_enabled) {
			HAL_Delay(50);
		}

		set_motor_speed(0, 0);
	}

	else if (space_found == 2 && self_parking_enabled) {  // Right-side parking
		// Reverse while steering right
		SERVO_voidRotate(htim2,SERVO_PWM, 135);
		HAL_Delay(100);

		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 0);
		set_motor_speed(60, 40);

		// Stop when back sensor reads < 15 cm
		while (get_distance(GPIOB, TRIG_BACK, ECHO_BACK) > 15  && self_parking_enabled) {
			HAL_Delay(50);
		}

		if (!self_parking_enabled) { set_motor_speed(0,0); BUZZER_OFF(); return; }
		set_motor_speed(0, 0);

		// Move forward and steer left to align
		SERVO_voidRotate(htim2,SERVO_PWM, 45);
		HAL_Delay(100);

		set_motor_direction(MOTOR1_GPIO_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
		set_motor_direction(MOTOR2_GPIO_PORT, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, 1);
		set_motor_speed(50, 50);

		while (get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT) > 20 && self_parking_enabled) {
			HAL_Delay(50);
		}

		set_motor_speed(0, 0);
	}




	SERVO_voidRotate(htim2,SERVO_PWM, 90);
	HAL_Delay(100);
	set_motor_speed(0, 0);

	// Disable parking mode
	self_parking_enabled = 0;

	BUZZER_OFF(); // Stop buzzer
	buzzer_beep(2, 200);
	// Send success message over UART

    //char msg[] = "✅ Parking Complete\r\n";
    //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}*/
// Check Parking Space
uint8_t check_parking_space() {

	if (get_distance(GPIOC, TRIG_LEFT1, ECHO_LEFT1) > 30 && get_distance(GPIOA, TRIG_LEFT2, ECHO_LEFT2) > 30) {
		return 1;  // Left side
	}
	if (get_distance(GPIOB, TRIG_RIGHT1, ECHO_RIGHT1) > 30 && get_distance(GPIOB, TRIG_RIGHT2, ECHO_RIGHT2) > 30) {
		return 2;  // Right side
	}
	return 0;
}
void buzzer_beep(uint8_t times, uint16_t delay_ms)
{
	for (uint8_t i = 0; i < times; i++) {
		BUZZER_ON();
		HAL_Delay(delay_ms);
		BUZZER_OFF();
		HAL_Delay(delay_ms);
	}
}
uint8_t check_collision_front() {
	uint16_t distance = get_distance(GPIOB, TRIG_FRONT, ECHO_FRONT);
	if (distance < COLLISION_DISTANCE_CM) {
		set_motor_speed(0, 0);
		buzzer_beep(3, 100);
		/*
        char warn_msg[] = "\r\n⚠�? Collision Detected! Stopping\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)warn_msg, strlen(warn_msg), 100);*/
		return 1;
	}
	return 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
