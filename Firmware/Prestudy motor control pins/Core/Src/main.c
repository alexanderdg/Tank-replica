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

CAN_HandleTypeDef hcan;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef RxMessage;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#define OVERVOLTAGE 29.2
#define ID 1

uint16_t adcBuffer[3];
uint16_t counter = 0;
uint8_t accl = 20;
uint8_t decl = 5;
uint32_t pwm_target_left = 400;
uint32_t pwm_target_right = 400;
uint32_t pwm_left = 400;
uint32_t pwm_right = 400;
uint8_t GD_FAULT = 0;
uint8_t OV_FAULT = 0;
uint8_t TIMEOUT = 0;
long timertick = 0;
long old_timertick_accl = 0;
long old_timertick_decl = 0;
long temp_timertick = 0;
long timeout_timertick = 0;
uint8_t speed_left = 0;
uint8_t speed_right = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_13);
}
*/

void resetTimeout() {
	timeout_timertick = HAL_GetTick();
}

float getInputVoltage()
{
	uint32_t temp = adcBuffer[0];
	uint32_t shuntVoltage = (801 * temp);
	float voltage = (shuntVoltage * 18.414) / 1000000;
	return voltage;
}

void sendACK()
{
	uint32_t mb;
	uint8_t data[] = {ID, 1};
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 2;
	TxMessage.TransmitGlobalTime = DISABLE;
	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, data, &mb) != HAL_OK) {
	    Error_Handler();
	}
}

void sendNACK()
{
	uint32_t mb;
	uint8_t data[] = {ID, 0};
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 2;
	TxMessage.TransmitGlobalTime = DISABLE;
	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, data, &mb) != HAL_OK) {
	    Error_Handler();
	}
}


void setPWMLeft(int PWM)
{
	pwm_left = PWM;
	uint32_t PWM_local = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	if(PWM >= 400)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		PWM_local = PWM - 400;
	}
	else if(PWM < 400)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		PWM_local = 400 - PWM;
	}
	int temp = PWM_local + 1;
	if(temp > 401)
	{
		temp = 401;
	}
	if(temp < 20)
	{
			temp = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, temp);
}

void setPWMRight(int PWM)
{
	pwm_right = PWM;
	uint32_t PWM_local = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	if(PWM >= 400)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		PWM_local = PWM - 400;
	}
	else if(PWM < 400)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		PWM_local = 400 - PWM;
	}
	int temp = PWM_local + 1;
	if(temp > 401)
	{
		temp = 401;
	}
	if(temp < 20)
	{
		temp = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, temp);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);
  uint32_t mb;
  uint32_t shuntVoltage;
  uint32_t temp_speed, temp_speed_left, temp_speed_right;
  uint8_t currentLSB, currentMSB, voltageLSB, voltageMSB;
  float current, voltage;
  uint8_t data[] = {ID,0,0,0,0,0,0,0};
  resetTimeout();
  switch(RxData[0])
  {
  	  case 0:
  		  //----------- Coast Brake -----------//
  		  //Set sleep pins low
  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  		  //Set Phase pins low
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  		  pwm_left = 400;
  		  pwm_target_left = 400;
  		  pwm_right = 400;
  		  pwm_target_right = 400;
  		  sendACK();
  		  break;
  	  case 1:
  		  //----------- Dynamic Brake (slow brake) -----------//
  		  //Set sleep pins high
  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  		  //Set Phase pins low
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  		  pwm_target_right = 400;
		  pwm_target_left = 400;
  		  setPWMLeft(400);
  		  setPWMRight(400);
  		  sendACK();
  		  break;
  	  case 2:
  		  //----------- Regenerative brake -----------//
  		  pwm_target_right = 400;
		  pwm_target_left = 400;
  		  sendACK();
  		  break;
  	  case 3:
  		  //----------- Forward drive -----------//
  		  if(RxData[1] > 800)
  		  {
  			  pwm_target_left = 800;
  			  pwm_target_right = 0;
  		  }
  		  else
  		  {
  			  pwm_target_left = 400 + RxData[1];
  			  pwm_target_right = 400 - RxData[1];
  		  }
  		  sendACK();
  		  break;
  	  case 4:
  		  //----------- Reverse drive -----------//
  		  if(RxData[1] > 400)
  		  {
  		  	  pwm_target_left = 0;
  		  	  pwm_target_right = 800;
  		  }
  		  else
  		  {
  			  pwm_target_left = 400 - RxData[1];
  		  	  pwm_target_right = 400 + RxData[1];
  		  }
  		  sendACK();
  		  break;
  	  case 5:
  		  //----------- Manual left motor -----------//
  		  temp_speed = (RxData[1] << 8) + RxData[2];
  		  if(temp_speed > 800) pwm_target_left = 800;
  		  else pwm_target_left = temp_speed;
  		  sendACK();
  		  break;
  	  case 6:
  		  //----------- Manual right motor -----------//
  		  temp_speed = (RxData[1] << 8) + RxData[2];
  		  if(temp_speed > 800) pwm_target_right = 800;
  		  else pwm_target_right = temp_speed;
  		  sendACK();
  		  break;
  	  case 7:
  		  //----------- Manual right/left motor -----------//
  		  temp_speed_left = (RxData[1] << 8) + RxData[2];
  		  temp_speed_right = (RxData[3] << 8) + RxData[4];
  		  if(temp_speed_left > 800) pwm_target_left = 800;
  		  else pwm_target_left = temp_speed_left;
  		  if(temp_speed_right > 800) pwm_target_right = 800;
  		  else pwm_target_right = temp_speed_right;
  		  sendACK();

  		  break;
  	  case 10:
  		  //----------- Set acceleration -----------//
  		  if(RxData[1] > 100) accl = 100;
  		  else accl = RxData[1];
  		  sendACK();
  		  break;
  	  case 11:
  		  //----------- Set deceleration -----------//
  		  if(RxData[1] > 100) decl = 100;
  		  else decl = RxData[1];
  		  sendACK();
  		  break;
  	  case 12:
  		  //----------- Set max torque -----------//
  		  if(RxData[1] > 100) DAC1->DHR12R1 = 4024;
  		  else DAC1->DHR12R1 = RxData[1] * 40;
  		  sendACK();
  		  break;
  	  case 13:
  		  //----------- Reset faults -----------//
  		  OV_FAULT = 0;
  		  GD_FAULT = 0;
  		  sendACK();
  		  break;
  	  case 100:
  		  //----------- Left current -----------//
  		  mb = adcBuffer[1];
  		  shuntVoltage = (801 * mb) / 20;
  		  current = (shuntVoltage / 10000.0);
  		  currentMSB = current;
  		  currentLSB = (current - currentMSB)*100;
  	  	  data[1] = currentMSB;
  	  	  data[2] = currentLSB;
  	  	  TxMessage.StdId = 0;
  	  	  TxMessage.IDE = CAN_ID_STD;
  	  	  TxMessage.RTR = CAN_RTR_DATA;
  	  	  TxMessage.DLC = 3;
  	  	  TxMessage.TransmitGlobalTime = DISABLE;
  	  	  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  	  		  Error_Handler();
  	  	  }
  		  break;
  	  case 101:
  		  //----------- Right current -----------//
  		  mb = adcBuffer[2];
  		  shuntVoltage = (801 * mb) / 20;
  		  current = (shuntVoltage / 10000.0);
  		  currentMSB = current;
  		  currentLSB = (current - currentMSB)*100;
  		  data[1] = currentMSB;
  		  data[2] = currentLSB;
  	  	  TxMessage.StdId = 0;
  	  	  TxMessage.IDE = CAN_ID_STD;
  	  	  TxMessage.RTR = CAN_RTR_DATA;
  	  	  TxMessage.DLC = 3;
  	  	  TxMessage.TransmitGlobalTime = DISABLE;
  	  	  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  	  		  Error_Handler();
  	  	  }
  		  break;
  	  case 102:
  		  //----------- Battery voltage -----------//
  		  voltage = getInputVoltage();
  		  voltageMSB = voltage;
  		  voltageLSB = (voltage - voltageMSB) * 100;
  		  data[1] = voltageMSB;
  		  data[2] = voltageLSB;
  		  TxMessage.StdId = 0;
  		  TxMessage.IDE = CAN_ID_STD;
  		  TxMessage.RTR = CAN_RTR_DATA;
  		  TxMessage.DLC = 3;
  		  TxMessage.TransmitGlobalTime = DISABLE;
  		  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  			  Error_Handler();
  		  }
  		  break;
  	  case 103:
  		  //----------- Left speed -----------//
  		  data[1] = speed_left;
  		  TxMessage.StdId = 0;
  		  TxMessage.IDE = CAN_ID_STD;
  		  TxMessage.RTR = CAN_RTR_DATA;
  		  TxMessage.DLC = 2;
  		  TxMessage.TransmitGlobalTime = DISABLE;
  		  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  		  	Error_Handler();
  		  }
  		  break;
  	  case 104:
  		  //----------- Right speed -----------//
  		  data[1] = speed_right;
  		  TxMessage.StdId = 0;
  		  TxMessage.IDE = CAN_ID_STD;
  		  TxMessage.RTR = CAN_RTR_DATA;
  		  TxMessage.DLC = 2;
  		  TxMessage.TransmitGlobalTime = DISABLE;
  		  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  		  	Error_Handler();
  		  }
  		  break;
  	  case 105:
  		  //----------- Get Status -----------//
  		  data[1] = OV_FAULT;
  		  data[2] = GD_FAULT;
  		  data[3] = TIMEOUT;
  		  TxMessage.StdId = 0;
  		  TxMessage.IDE = CAN_ID_STD;
  		  TxMessage.RTR = CAN_RTR_DATA;
  		  TxMessage.DLC = 4;
  		  TxMessage.TransmitGlobalTime = DISABLE;
  		  if (HAL_CAN_AddTxMessage(hcan, &TxMessage, data, &mb) != HAL_OK) {
  		    Error_Handler();
  		  }
  		  break;
  };
  counter ++;

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void HAL_SYSTICK_Callback(void)  {

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 3);
  HAL_NVIC_SetPriority(CAN_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
  //SysTick_Config(SystemCoreClock / 800000);
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/800000000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  DAC1->DHR12R1 = 4000;
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  timertick = HAL_GetTick();
	  if(OV_FAULT == 1)
	  {
		  setPWMLeft(400);
	  	  setPWMRight(400);
	   	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	   	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	   	  HAL_Delay(100);
	  }
	  else if(TIMEOUT == 1)
	  {
		  //Set sleep pins low
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		  //Set Phase pins low
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		  pwm_left = 400;
		  pwm_target_left = 400;
		  pwm_right = 400;
		  pwm_target_right = 400;
		  HAL_Delay(100);
	  }
	  else if(pwm_target_left != pwm_left || pwm_target_right != pwm_right)
	  {
		  if((timertick - old_timertick_accl) >= accl)
		  {
			  old_timertick_accl = timertick;
			  if(pwm_left < 400)
			  {
				  if(pwm_target_left < pwm_left)
				  {
					  if((pwm_left - pwm_target_left) > 10) setPWMLeft(pwm_left - 10);
					  else setPWMLeft(pwm_left - 1);
				  }
			  }
			  else
			  {
				  if(pwm_target_left > pwm_left)
				  {
					  if((pwm_target_left - pwm_left) > 10) setPWMLeft(pwm_left + 10);
					  else setPWMLeft(pwm_left + 1);
				  }
			  }
			  if(pwm_right < 400)
			  {
				  if(pwm_target_right < pwm_right)
				  {
					  if((pwm_right - pwm_target_right) > 10) setPWMRight(pwm_right - 10);
					  else setPWMRight(pwm_right - 1);
				  }
			  }
			  else
			  {
				  if(pwm_target_right > pwm_right)
				  {
					  if((pwm_target_right - pwm_right) > 10) setPWMRight(pwm_right + 10);
					  else setPWMRight(pwm_right + 1);
				  }
			  }
		  }
		  if((timertick - old_timertick_decl) >= decl)
		  {
			  old_timertick_decl = timertick;
			  if(pwm_left < 400)
			  {
				  if(pwm_target_left > pwm_left)
				  {
					  if((pwm_target_left - pwm_left) > 10) setPWMLeft(pwm_left + 10);
					  else setPWMLeft(pwm_left + 1);
				  }
			  }
			  else
			  {
				  if(pwm_target_left < pwm_left)
				  {
				  	  if((pwm_left - pwm_target_left) > 10) setPWMLeft(pwm_left - 10);
				  	  else setPWMLeft(pwm_left - 1);
				  }
			  }
			  if(pwm_right < 400)
			  {
				  if(pwm_target_right > pwm_right)
				  {
				  	  if((pwm_target_right - pwm_right) > 10) setPWMRight(pwm_right + 10);
				  	  else setPWMRight(pwm_right + 1);
				  }
			  }
			  else
			  {
				  if(pwm_target_right < pwm_right)
				  {
					  if((pwm_right - pwm_target_right) > 10) setPWMRight(pwm_right - 10);
					  else setPWMRight(pwm_right - 1);
				  }
			  }
		  }
  	  }
  	  float inputVoltage = getInputVoltage();
  	  if(inputVoltage > OVERVOLTAGE && OV_FAULT == 0)
  	  {
  		  OV_FAULT = 1;
  	  }
  	  if((!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) || !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)))
  	  {
  		  GD_FAULT = 1;
  	  }
  	  else{
  		  GD_FAULT = 0;
  	  }
  	  if((timertick - timeout_timertick) >= 5000)
  	  {
  		  TIMEOUT = 1;
  	  }
  	  else {
  		  TIMEOUT = 0;
  	  }
  	  if((timertick - temp_timertick) >= 100)
  	  {
  		  speed_left = TIM1 -> CNT;
  		  speed_right = TIM2 -> CNT;
  		  TIM1 -> CNT = 0;
  		  TIM2 -> CNT = 0;
  		  temp_timertick = timertick;
  	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T15_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef sf;
    sf.FilterMaskIdHigh = 0xFFFF;
    sf.FilterMaskIdLow = 0;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sf.FilterBank = 0;
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterActivation = CAN_FILTER_ENABLE;
    sf.FilterIdLow= 0;
    sf.FilterIdHigh = 0x001 << 5;
    if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
    }



  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 200;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
