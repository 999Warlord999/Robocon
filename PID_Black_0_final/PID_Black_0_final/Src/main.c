/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Address 0
#define enc TIM3-> CNT
#define Speed_Forward 1
#define Speed_Direction 2
#define Position_Forward 3
#define Position_Direction 4

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int16_t debug;
uint8_t mode;
uint16_t setpoint;
uint16_t Kp;
uint8_t Ki;
float Kd;
	
char SignalControl[30];

int8_t flagPID = 0;
int8_t flagPIDposition = 1;
int8_t flagUART = 0;

int16_t outPID;
int16_t outPID1;
int16_t encoder = 0;
int16_t Tim2Period;
float speed;
float T;
float anpha;
float beta; 
float gama; 
float delta; 
float deltaPWM;
float e;
float e1;
float e2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_init()
{
	Tim2Period = 3599;
	T = 0.001;
	deltaPWM = (float)(Tim2Period*0.3)/100;
}
void OutPWM(int16_t duty)
{
	if (duty >= (Tim2Period - deltaPWM)) 
		duty = Tim2Period - deltaPWM;
	else if (duty <= -(Tim2Period - deltaPWM)) 
		duty = -(Tim2Period - deltaPWM);
	if (duty > 0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Tim2Period - duty);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Tim2Period);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Tim2Period);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Tim2Period + duty);
	}
}
void Compute_positionPID(int16_t set, float Kp, float Ki, float Kd)
{
	if(flagPID){	
		if(set > 0){
			flagPID = 0;
			anpha = 2*T*Kp + Ki*T*T + 2*Kd;
			beta = T*T*Ki - 4*Kd - 2*T*Kp;
			gama = 2*Kd;
			delta = 2*T;
			e2 = e1;
			e1 = e;
			e = set - encoder;
			outPID1 = outPID;
			outPID = (anpha*e + beta*e1 + gama*e2 + delta*outPID1) / delta;
			OutPWM(outPID);
		}
		else{
			flagPID = 0;
			anpha = 2*T*Kp + Ki*T*T + 2*Kd;
			beta = T*T*Ki - 4*Kd - 2*T*Kp;
			gama = 2*Kd;
			delta = 2*T;
			e2 = e1;
			e1 = e;
			e = -(set - encoder);
			outPID1 = outPID;
			outPID = (anpha*e + beta*e1 + gama*e2 + delta*outPID1) / delta;
			OutPWM(-outPID);
		}
	}
}

void speedPID(float set, float Kp, float Ki, float Kd)
{
	if(flagPID){	
		if(set > 0){
			flagPID = 0;
			anpha = 2*T*Kp + Ki*T*T + 2*Kd;
			beta = T*T*Ki - 4*Kd - 2*T*Kp;
			gama = 2*Kd;
			delta = 2*T;
			e2 = e1;
			e1 = e;
			e = set - encoder;
			enc = 0;
			outPID1 = outPID;
			outPID = (anpha*e + beta*e1 + gama*e2 + delta*outPID1) / delta;
			OutPWM(outPID);
		}
		else{
			flagPID = 0;
			anpha = 2*T*Kp + Ki*T*T + 2*Kd;
			beta = T*T*Ki - 4*Kd - 2*T*Kp;
			gama = 2*Kd;
			delta = 2*T;
			e2 = e1;
			e1 = e;
			e = -(set - encoder);
			speed = (encoder * 1000) / 400;
			enc = 0;
			outPID1 = outPID;
			outPID = (anpha*e + beta*e1 + gama*e2 + delta*outPID1) / delta;
			OutPWM(-outPID);
		}
	}
}

void positionPID(int16_t set)
{
	if(set > 0){
		if((encoder < set - 400) && (flagPIDposition == 1))
			OutPWM(3600);
		else if (flagPIDposition == 1)
			flagPIDposition = 2;
		if(flagPIDposition == 2){
			flagPIDposition = 3;
			enc = 0;
		}
		if(flagPIDposition == 3){
			Compute_positionPID(400, 11, 2, 0.8);
		}
	}
	else{
		if((-encoder < (-set) - 400) && (flagPIDposition == 1))
			OutPWM(-3600);
		else if (flagPIDposition == 1)
			flagPIDposition = 2;
		if(flagPIDposition == 2){
			flagPIDposition = 3;
			enc = 0;
		}
		if(flagPIDposition == 3){
			Compute_positionPID(-400, 11, 2, 0.8);
		}
	}
}

void resetPID()
{
	enc = 0;
	encoder = 0;
	e = 0;
	e1 = 0;
	e2 = 0;
	outPID1 = 0;
	outPID = 0;
}

void RefeshSignal()
{
	if (flagUART)
	{
		flagUART = 0;
		if ( Address < 3)
		{ 
			mode = SignalControl[Address*7];
			
			setpoint = SignalControl[Address*7 + 2];
			setpoint = setpoint << 8;
			setpoint = setpoint + SignalControl[Address*7 + 1];
			
			Kp = SignalControl[Address*7 + 4];
			Kp = Kp << 8;
			Kp = Kp + SignalControl[Address*7 + 3];
			
			Ki = SignalControl[Address*7 + 5];
			Kd = SignalControl[Address*7 + 6];
			Kd = Kd/100;
		}
		else
		{
			mode = SignalControl[(Address+4)*3];

			setpoint = SignalControl[(Address+4)*3+2];
			setpoint = setpoint << 8;
			setpoint = setpoint + SignalControl[(Address+4)*3+1];
			
			Kp = 0;
			Ki = 0;
			Kd = 0;
		}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*)SignalControl, 30);
	PID_init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_3|TIM_CHANNEL_4);	
	HAL_TIM_Base_Start_IT(&htim1);
	//hal_tim_pwm_
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		/* main code*/
//		RefeshSignal();
//		if(mode == Position_Forward)
//			positionPID(setpoint);
//		else if (mode == Position_Direction)
//			positionPID(-setpoint);
//		else if (mode == Speed_Forward)
//			speedPID(setpoint, Kp, Ki, Kd);
//		else if(mode == Speed_Direction)
//			speedPID(-setpoint, Kp, Ki, Kd);	
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		flagPID = 1;
		encoder = enc;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_UART_Receive_IT(&huart1, (uint8_t*)SignalControl, 30);
		flagUART = 1;
		flagPIDposition = 1;
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
