/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ModBusRTU.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];
uint16_t VacuumState;
uint16_t GripperState;
uint16_t GoalPoint;
uint16_t target_point ;
uint16_t PickOrder ;
uint16_t PlaceOrder ;
uint16_t Pick = 0 ;
uint16_t Place = 0 ;
uint64_t _micros;
uint64_t a = 0;
uint16_t status1 = 0 ;
uint16_t reset = 0 ;
uint16_t B1 = 0 ;
uint16_t Set = 0 ;
uint16_t ReSet = 0 ;
int first = 0;
int second = 0 ;
int third = 0 ;
int fourth = 0;
int fifth = 0 ;

int arrayPick[5];
int arrayPlace[5];



//static uint32_t timestamp2 = 0;
//int64_t currentTime = 0;

enum state{

	Heartbeat1,
//	Routine1,
	Vacuum1,
	Gripper_Movement1,
	set_shelves1,
	setGoalPoint1,
	run_pointmode1,
	set_home1,
	setPick_PlaceOrder1,
	run_jog_mode1
};


int mode = 0 ;
int pinState = 0 ;
int x = 0 ;
int i = 0 ;
int y = 0 ;
int z = 0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Heartbeat ();
void Routine() ;
void Vacuum();
void Gripper_Movement();
void set_shelves();
void setGoalPoint();
void run_pointmode ();
void set_home ();
void setPick_PlaceOrder();
void run_jog_mode();
uint64_t micros();
void Loop();
void Reset();
void ConvertToArray();


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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =200;
  Modbus_init(&hmodbus, registerFrame);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Set = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) ;
	  ReSet = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		if (ReSet == 1) {
			for (int l = 0; l < 5; l++) {
				registerFrame[0x23 + l].U16 = 0;
				i = 0;
			}
		}

		Modbus_Protocal_Worker();
//		Loop();
		Heartbeat();
		Routine();
		Vacuum();
		Gripper_Movement();
		set_shelves();
		setGoalPoint();
		run_pointmode();
		set_home();
		setPick_PlaceOrder();
		run_jog_mode();
		ConvertToArray();


		//Vacuum
//			if (registerFrame[0x02].U16 == 0b0000) {
//				VacuumState = 0; //Off
//			}
//			if (registerFrame[0x02].U16 == 0b0001) {
//				VacuumState = 1; //On
//			}
//
//
		//Gripper_Movement
//		if (registerFrame[0x03].U16 == 0b0000) {
//			GripperState = 0; //Backward
//		}
//		if (registerFrame[0x03].U16 == 0b0001) {
//			GripperState = 1; //Forward
//		}
//
//
		//set_shelves
//		if (registerFrame[0x01].U16 == 1) {
//
////				registerFrame[0x01].U16 = 0; //reset base status
//				registerFrame[0x10].U16 = 1; // update z axis status
//
//				// �?ำหนดตำ�?หน่ง�?ต่ละชั้น
//				registerFrame[0x23].U16 = 5000;
//				registerFrame[0x24].U16 = 3000;
//				registerFrame[0x25].U16 = 4000;
//				registerFrame[0x26].U16 = 2000;
//				registerFrame[0x27].U16 = 1000;
//
//				if(status == 0){
//					status = 1 ;
//					registerFrame[0x01].U16 = 1;
//				}
//
//
//				status = 1;
//				registerFrame[0x10].U16 = 0; // reset z axis status
//		}
//
//
//
		//setGoalPoint
//		if (registerFrame[0x01].U16 == 8) { // if run point mode
//			GoalPoint = (registerFrame[0x30].U16) / 10; // หาร 10 เพราะเป็นค่าที่มาจา�? base
//		}
//
		//run_pointmode
//		if (registerFrame[0x01].U16 == 8) {
//			registerFrame[0x01].U16 = 0;
//			registerFrame[0x10].U16 = 16;
//			registerFrame[0x30].U16 = target_point;
//			registerFrame[0x10].U16 = 0;
//		}
//
		//set_home
//		if (registerFrame[0x01].U16 == 0b0010) {
//			registerFrame[0x01].U16 = 0b0000;
//			registerFrame[0x10].U16 = 0b0010;
//			target_point = 0;
//			registerFrame[0x10].U16 = 0b0000;
//		}
//
		//setPick_PlaceOrder
//		if (registerFrame[0x01].U16 == 4) {
//			PickOder = registerFrame[0x21].U16; // ค่าชั้นที่ต้อง Pick
//			PlaceOder = registerFrame[0x22].U16; // ค่าชั้นที่ต้อง Place
//			//ค่าที่ได้จะเรียงติด�?ัน ex.ถ้าเซ็ตค่าในUIชั้น�?ร�?ที่ต้อง Pick คือ ชั้น1-5 ตามลำดับ ค่าชั้นที่ต้องPick จะได้ 12345
//		}
//
		//run_jog_mode
//		if (registerFrame[0x01].U16 == 0b0100) {
//			registerFrame[0x01].U16 = 0b0000;
//			// loop หยิบจับ 5 รอบ
//			registerFrame[0x10].U16 = 0b0100;
//			//condition ตำ�?หน่งที่จะไปหยิบ
//
//			registerFrame[0x10].U16 = 0b1000;
//			//		//condition ตำ�?หน่งที่จะไปวาง
//
//			registerFrame[0x10].U16 = 0b0000;
//		}


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1145;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void Loop(state){
//	static uint32_t timestamp = 0;
//
//	switch (state){
//	case Heartbeat1:
//			mode = 1;
//			if (timestamp < HAL_GetTick()) {
//				timestamp = HAL_GetTick() + 200;
//				registerFrame[0x00].U16 = 22881;
//				a += 1;
//
//
//			if (registerFrame[0x00].U16 == 18537) {
//				registerFrame[0x04].U16 = 0b0001;  // lead1 = off , lead2 = on
//				registerFrame[0x10].U16 = status1; // ต้องสร้างตัว�?ปรมาใส่
//
//				registerFrame[0x11].U16 = 100 * 10; // ค่าที่ได้จา�? control ต้อง *10
//				registerFrame[0x12].U16 = 100 * 10; // ค่าที่ได้จา�? control ต้อง *10
//				registerFrame[0x13].U16 = 100 * 10; // ค่าที่ได้จา�? control ต้อง *10
//				registerFrame[0x40].U16 = 100 * 10; // ค่าจา�?�?�?น X (ต้องคูณ 10 )
//			}
//			}
//
//
//		break;
//
//	case Vacuum1 :
//		mode = 3 ;
//		if (registerFrame[0x02].U16 == 0b0000) {
//			VacuumState = 0; //Off
//		}
//		if (registerFrame[0x02].U16 == 0b0001) {
//			VacuumState = 1; //On
//		}
//
//
//		break;
//
//	case Gripper_Movement1 :
//		mode = 4 ;
//		if (registerFrame[0x03].U16 == 0b0000) {
//			GripperState = 0; //Backward
//		}
//		if (registerFrame[0x03].U16 == 0b0001) {
//			GripperState = 1; //Forward
//		}
//
//
//		break;
//
//	case set_shelves1 :
//		mode = 5 ;
//		if (registerFrame[0x01].U16 == 1) {
//
//			registerFrame[0x01].U16 = 0; //reset base status
//			registerFrame[0x10].U16 = 1; // update z axis status
//			status1 = 1;
//
//			first = 1000;
//			second = 2000;
//			third = 3000;
//			fourth = 4000;
//			fifth = 5000;
//
//			B1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//			if (B1 == 1) {
//				registerFrame[0x01].U16 = 0;
//				registerFrame[0x10].U16 = 0;
//				status1 = 0;
////				read position every shelves
//				registerFrame[0x23].U16 = first;
//				registerFrame[0x24].U16 = second;
//				registerFrame[0x25].U16 = third;
//				registerFrame[0x26].U16 = fourth;
//				registerFrame[0x27].U16 = fifth;
//			}
//		}
//
//		break;
//
//	case setGoalPoint1:
//		mode = 6 ;
//		if (registerFrame[0x01].U16 == 8) { // if run point mode
//			GoalPoint = (registerFrame[0x30].U16) / 10; // divide 10 because data from base system
//		}
//
//		break;
//
//	case run_pointmode1 :
//		mode = 7 ;
//		if (registerFrame[0x01].U16 == 8) {
//			registerFrame[0x01].U16 = 0;
//			registerFrame[0x10].U16 = 16; // go point
//			status1 = 16 ;
//			registerFrame[0x30].U16 = target_point; // sent data go point
//			registerFrame[0x10].U16 = 0;
//			status1 = 0 ;
//		}
//
//		break;
//
//	case set_home1 :
//		mode = 8 ;
//		if (registerFrame[0x01].U16 == 0b0010) {
//			registerFrame[0x01].U16 = 0b0000;
//			registerFrame[0x10].U16 = 0b0010;
//			status1 = 2 ;
//			target_point = 0;
//			registerFrame[0x10].U16 = 0b0000;
//			status1 = 0 ;
//		}
//
//		break;
//	case setPick_PlaceOrder1 :
//		mode = 9 ;
//		if (registerFrame[0x01].U16 == 4) {
//			PickOder = registerFrame[0x21].U16; // ค่าชั้นที่ต้อง Pick
//			PlaceOder = registerFrame[0x22].U16; // ค่าชั้นที่ต้อง Place
//			//result -> pick ex. 54321 (type = int)
//		}
//
//		break;
//	case run_jog_mode1:
//		mode = 10 ;
//		if (registerFrame[0x01].U16 == 0b0100) {
//			registerFrame[0x01].U16 = 0b0000;
//			// loop หยิบจับ 5 รอบ
//			registerFrame[0x10].U16 = 0b0100;
//			status1 = 4 ;
//			//condition ตำ�?หน่งที่จะไปหยิบ
//
//			registerFrame[0x10].U16 = 0b1000;
//			status1 = 8;
//			//		//condition ตำ�?หน่งที่จะไปวาง
//
//			registerFrame[0x10].U16 = 0b0000;
//			status1 = 16 ;
//		}
//
//		break;
//}
//}



//uint64_t micros()
//{
//	return __HAL_TIM_GET_COUNTER(&htim2) + _micros;
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim2) {
//		_micros += UINT32_MAX;
//	}
//}

void Heartbeat (){
	static uint32_t timestamp = 0;
	if (timestamp < HAL_GetTick()) {
		timestamp = HAL_GetTick() + 200;
		registerFrame[0x00].U16 = 22881;

	}
}

void Routine() {
	static uint32_t timestamp = 0;
	if (timestamp < HAL_GetTick()) {
		timestamp = HAL_GetTick() + 200;
		if (registerFrame[0x00].U16 == 18537) {
//			Reed_Out = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7); // set pull up, if on = 0
//			Reed_In = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8); // set pull up, if on = 0
//			if(Reed_Out == 0){
				registerFrame[0x04].U16 = 0b0000;  // Reed 1 = off , Reed 2 = on
//			}
//			else if (Reed_In == 0){
//				registerFrame[0x04].U16 = 0b0010;  // Reed 1 = on , Reed 2 = off
//			}
			registerFrame[0x10].U16 = status1; // z - axis status * 10
			registerFrame[0x11].U16 = 100 * 10; // z - axis position *10
			registerFrame[0x12].U16 = 100 * 10; // z - axis speed  *10
			registerFrame[0x13].U16 = 100 * 10; // z - axis acceleration *10
			registerFrame[0x40].U16 = 100 * 10; // x - axis position * 10
		}
	}
}

void Vacuum(){ // อ่านค่า Vacuum from BaseSytem
	if(registerFrame[0x02].U16 == 0b0000){
		VacuumState = 0; //Off
	}
	if(registerFrame[0x02].U16 == 0b0001){
		VacuumState = 1; //On
		}
}

void Gripper_Movement(){ // อ่านค่า Gripper from BaseSytem
	if(registerFrame[0x03].U16 == 0b0000){
			GripperState = 0; //Backward
		}
	if(registerFrame[0x03].U16 == 0b0001){
			GripperState = 1; //Forward
			}
}

void set_shelves() {
	if (registerFrame[0x01].U16 == 1) {
//		registerFrame[0x01].U16 = 0b0000; //reset base status
		registerFrame[0x10].U16 = 0b0001; // update z axis status
		status1 = 1 ;
		y += 1;
		if (Set == 1) {
			static uint32_t TimeStamp = 0;
			if (HAL_GetTick() > TimeStamp) {
				TimeStamp = HAL_GetTick() + 500;
				registerFrame[0x23 + i].U16 = 1000 * i;
				i += 1;
			}
		}
		else if (i > 4)
		{
			i = 0;
			registerFrame[0x01].U16 = 0b00000;
			registerFrame[0x10].U16 = 0b00000;
			status1 = 0;
		}
	}
}

void setGoalPoint(){
	if(registerFrame[0x01].U16 == 8){ // if run point mode
		 GoalPoint = (registerFrame[0x30].U16)/10 ; // divide 10 because data from base system
	}
}



void run_pointmode() {
	if (registerFrame[0x01].U16 == 8) {
//		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = 16;
		status1 = 16 ;
		B1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (B1 == 1) {
			registerFrame[0x01].U16 = 0;
			registerFrame[0x10].U16 = 0;
			status1 = 0;
		}
	}
}

void set_home() {
	if (registerFrame[0x01].U16 == 0b0010) {
//		registerFrame[0x01].U16 = 0b0000;
		registerFrame[0x10].U16 = 0b0010;
		status1 = 2;
		GoalPoint = 0;

		B1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (B1 == 1) {
			registerFrame[0x01].U16 = 0b0000;
			registerFrame[0x10].U16 = 0b0000;
			status1 = 0;
		}
	}
}

void setPick_PlaceOrder(){
	if (registerFrame[0x01].U16 == 4) {
		PickOrder = registerFrame[0x21].U16; // ค่าชั้นที่ต้อง Pick
		PlaceOrder = registerFrame[0x22].U16; // ค่าชั้นที่ต้อง Place
	}
}

void ConvertToArray(){
	Pick = PickOrder ;
	Place = PlaceOrder ;
 	for (int i = 4; i >= 0; i--) {  // convert Pick from int to array
		arrayPick[i] = Pick % 10;
		Pick /= 10;
	}

	for (int i = 4; i >= 0; i--) {  // convert Place from int to array
		arrayPlace[i] = Place % 10;
		Place /= 10;
	}
}


void run_jog_mode() {
	if (registerFrame[0x01].U16 == 0b0100) {
//		registerFrame[0x01].U16 = 0b0000;
		// loop หยิบจับ 5 รอบ
		registerFrame[0x10].U16 = 0b0100;
		status1 = 4 ;
		//condition ตำ�?หน่งที่จะไปหยิบ

//		registerFrame[0x10].U16 = 0b1000;
//		status1 = 8 ;
//		//condition ตำ�?หน่งที่จะไปวาง


		B1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (B1 == 1) {
			registerFrame[0x01].U16 = 0b0000;
			registerFrame[0x10].U16 = 0b0000;
			status1 = 0;
		}

	}
}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	x += 1;
//	if (GPIO_Pin == GPIO_PIN_10) {
//		if (mode == 0)
//		{
//			mode = 1;
//		}
//		else
//		{
//			mode = 0;
//		}
//	}
//}
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
