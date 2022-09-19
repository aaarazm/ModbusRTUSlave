/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t coils[9999];
uint16_t holdingRegs[9999];
uint16_t inputRegs[9999];
uint8_t inputStat[9999];

static uint8_t myAddress = 0x01;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

CRC_HandleTypeDef hcrc;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t result;
int countinCycles1 = 0;
int countinCycles2 = 0;
int countinCycles3 = 0;
int countinCycles4 = 0;
int timerFlag = 0;
int RxDone, TxDone;
uint8_t frame[256];
uint8_t RxBuffer[256];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

void initial1bits(uint8_t* mem);
void initial16bits(uint8_t* mem);
uint16_t stick(uint8_t msb, uint8_t lsb);
int biggerMark8Coils(uint16_t count);
int RxCRCcheck(uint8_t* frame, int type);
uint16_t CRCGen(uint8_t* frame, int type);
int transSize(uint8_t* frame);
void func1(uint8_t frame[256], uint8_t RxBuffer[256]);
void func2(uint8_t frame[256], uint8_t RxBuffer[256]);
void func3(uint8_t frame[256], uint8_t RxBuffer[256]);
void func4(uint8_t frame[256], uint8_t RxBuffer[256]);
void func5(uint8_t frame[256], uint8_t RxBuffer[256]);
void func6(uint8_t frame[256], uint8_t RxBuffer[256]);
void func15(uint8_t frame[256], uint8_t RxBuffer[256]);
void func16(uint8_t frame[256], uint8_t RxBuffer[256]);
void call_functions(uint8_t frame[256], uint8_t RxBuffer[256]);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	enum uartStates { receive1, wait1, checkingFrame, receive2, wait2, framing, transmit };
	enum uartStates pstate1 = receive1;
	int pstate2 = 0;
	int receiveCount = 0;
	int transmitSize;
	int Dibs = 1;
	//int receive2pass = 0;
	RxDone = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  initial1bits(coils);
  initial1bits(inputStat);
  initial16bits(holdingRegs);
  initial16bits(inputRegs);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)) {
		  __HAL_TIM_SET_COUNTER(&htim6,0);
	  }

/*	  if ((timerFlag == 1) & (Dibs == 1)) {
		  Dibs = 0;
		  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	  }*/

	  switch(pstate1)
	  {
	  case receive1:
		  receiveCount = 0;
		  if ((RxDone == 0) & (timerFlag == 1)) {
			  timerFlag = 0;
			  HAL_UART_Receive_DMA(&huart2, (uint8_t *)frame, 8);
			  pstate1 = wait1;
		  }
		  else {
			  pstate1 = receive1;
		  }
		  break;
	  case wait1:
		  if(RxDone == 1) {
			  RxDone = 0;
			  pstate1 = checkingFrame;
		  }
		  else {
			  pstate1 = wait1;
		  }
		  break;
	  case checkingFrame:
		  if ((frame[0] == myAddress) | (frame[0] == 0x00)) {
			  if ((frame[1] == 0x01) | (frame[1] == 0x02) | (frame[1] == 0x03) | (frame[1] == 0x04) | (frame[1] == 0x05) | (frame[1] == 0x06)) {
				  receiveCount = 0; // functions with fixed size
			  	  pstate1 = framing;
			  }
			  else if ((frame[1] == 0x0f) | (frame[1] == 0x10)) {
				  if (frame[6] < 248) {
					  receiveCount = 1 + frame[6]; // functions with dynamic size
					  pstate1 = receive2;
				  }
				  else {
					  countinCycles2++;
					  pstate1 = receive1;
				  }
			  }
			  else {
				  Dibs = 1;
				  pstate1 = receive1; //invalid function
			  }

		  }
		  else {
			  Dibs = 1;
			  receiveCount = -1;
			  pstate1 = receive1; //not my address
		  }
		  break;
	  case receive2:
		  HAL_UART_Receive_DMA(&huart2, (uint8_t *)(frame + 8), receiveCount);
		  pstate1 = wait2;
		  break;
	  case wait2:
		  if (RxDone == 1) {
			  RxDone = 0;
			  pstate1 = framing;
		  }
		  else
			  pstate1 = wait2;
		  break;
	  case framing:
		  call_functions(frame, RxBuffer);
		  pstate1 = transmit;
		  break;
	  case transmit:
		  if (RxBuffer[1] == 128 + frame[1]) {
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)RxBuffer, 8);
		  }
		  else if (RxBuffer[1] == frame[1]) {
			  transmitSize = transSize(RxBuffer);
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)RxBuffer, transmitSize);
		  }
		  Dibs = 1;
		  countinCycles1++;
		  pstate1 = receive1;
	  default:
		  pstate1 = receive1;
		  break;
	  }
/*	  switch(pstate2) //checking the concurrency
	  {
	  case 0:
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
			  pstate2 = 1;
		  }
		  else {
			  pstate2 = 0;
		  }
		  break;
	  case 1:
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
			  pstate2 = 2;
		  }
		  else {
			  pstate2 = 1;
		  }
		  break;
	  case 2:
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
			  pstate2 = 0;
		  }
		  else {
			  pstate2 = 2;
		  }
		  break;
	  default:
		  pstate2 = 0;
		  break;
	  }*/
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 32773;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0xffff;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void initial1bits(uint8_t* mem) {
    for (int i = 0; i < 9999; i++) {
        mem[i] = rand() & 0x01;
    }
}

void initial16bits(uint8_t* mem) {
    for (int i = 0; i < 9999; i++) {
        mem[i] = (rand() * 2) & 0xffff;
    }
}

uint16_t stick(uint8_t msb, uint8_t lsb) {
    uint16_t result, MSB;
    MSB = msb;
    result = (MSB << 8) + lsb;
    return result;
}

int biggerMark8Coils(uint16_t count) {
    int bytecount;
    bytecount = count / 8;
    if ((count % 8) != 0) {
        bytecount++;
    }
    return bytecount;
}

int RxCRCcheck(uint8_t* frame, int type) {
    int count, flag;
    uint32_t result;
    switch (type)
    {
    case 1:
        count = 6;
        break;
    case 2:
        count = frame[6] + 7;
        break;
    default:
        count = 6;
    }
    result = HAL_CRC_Calculate(&hcrc, frame, count);
    if (result == stick(frame[count + 1], frame[count]))
        flag = 0;
    else
        flag = 1;
    return flag;
}

uint16_t CRCGen(uint8_t* frame, int type) {
    int count;
    uint16_t result;
    switch (type)
    {
    case 1:
        count = 3 + frame[2];
        break;
    case 2:
        count = 6;
        break;
    default:
        count = 6;
    }
    result = HAL_CRC_Calculate(&hcrc, frame, count);
    return result;
}

int transSize(uint8_t* frame) {
    int size;
    if ((frame[1] == 0x01) | (frame[1] == 0x02) | (frame[1] == 0x03) | (frame[1] == 0x04)) {
        size = 5 + frame[2];
    }
    else {
        size = 8;
    }
    return size;
}

void func1(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = biggerMark8Coils(count);
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 2000) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            for (int j = 0; j < RxBuffer[2]; j++) {
                RxBuffer[3 + j] = 0x00;
            }
            for (int i = 0; i < count; i++) {
                RxBuffer[3 + (i / 8)] += (coils[start + i] << (i % 8));
            }
        }
        crcResult = CRCGen(RxBuffer, 1);
        RxBuffer[3 + RxBuffer[2]] = (crcResult & 0xff);
        RxBuffer[4 + RxBuffer[2]] = ((crcResult >> 8) & 0xff);
        for (int i = 0; i < (5 + RxBuffer[2]); i++) {
            printf("Buffer[%d]: %02x\n", i, RxBuffer[i]); //For testing.
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
    	for(int k = 2; k < 6; k++)
    		RxBuffer[k] = frame[k];
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func2(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = biggerMark8Coils(count);
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 2000) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            for (int j = 0; j < RxBuffer[2]; j++) {
                RxBuffer[3 + j] = 0x00;
            }
            for (int i = 0; i < count; i++) {
                RxBuffer[3 + (i / 8)] += (inputStat[start + i] << (i % 8));
            }
        }
        crcResult = CRCGen(RxBuffer, 1);
        RxBuffer[3 + RxBuffer[2]] = (crcResult & 0xff);
        RxBuffer[4 + RxBuffer[2]] = ((crcResult >> 8) & 0xff);
        for (int i = 0; i < (5 + RxBuffer[2]); i++) {
            printf("Buffer[%d]: %02x\n", i, RxBuffer[i]); //For testing.
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
    	for(int k = 2; k < 6; k++)
    		RxBuffer[k] = frame[k];
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func3(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = count * 2;
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 125) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            for (int j = 0; j < RxBuffer[2]; j++) {
                RxBuffer[3 + j] = 0x00;
            }
            for (int i = 0; i < count; i++) {
                RxBuffer[3 + (i * 2)] += ((holdingRegs[start + i] >> 8) & 0xff);
                RxBuffer[4 + (i * 2)] += (holdingRegs[start + i] & 0xff);
            }
        }
        crcResult = CRCGen(RxBuffer, 1);
        RxBuffer[3 + (count * 2)] = (crcResult & 0xff);
        RxBuffer[4 + (count * 2)] = ((crcResult >> 8) & 0xff);
        for (int i = 0; i < (5 + RxBuffer[2]); i++) {
            printf("(function's local) Buffer[%d]: %02x\n", i, RxBuffer[i]); //For testing.
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
    	for(int k = 2; k < 6; k++)
    		RxBuffer[k] = frame[k];
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func4(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = count * 2;
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 125) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            for (int j = 0; j < RxBuffer[2]; j++) {
                RxBuffer[3 + j] = 0x00;
            }
            for (int i = 0; i < count; i += 2) {
                RxBuffer[3 + (i * 2)] += ((inputRegs[start + i] >> 8) & 0xff);
                RxBuffer[4 + (i * 2)] += (inputRegs[start + i] & 0xff);
            }
        }
        crcResult = CRCGen(RxBuffer, 1);
        RxBuffer[3 + (count * 2)] = (crcResult & 0xff);
        RxBuffer[4 + (count * 2)] = ((crcResult >> 8) & 0xff);
        for (int i = 0; i < (5 + RxBuffer[2]); i++) {
            printf("(function's local) Buffer[%d]: %02x\n", i, RxBuffer[i]); //For testing.
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
    	for(int k = 2; k < 6; k++)
    		RxBuffer[k] = frame[k];
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func5(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int forced, address, crcflag;
    uint16_t crcResult;
    address = stick(frame[2], frame[3]);
    forced = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = frame[2];
    RxBuffer[3] = frame[3];
    RxBuffer[4] = frame[4];
    RxBuffer[5] = frame[5];
    if (crcflag == 0) {
        if ((address < 0) | (address > 9998)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            if (forced == 0xff00) {
                coils[address] = 0x01;
            }
            else if (forced == 0x0000) {
                coils[address] = 0x00;
            }
            else {
                printf("Invalid instruction, 0xff00 to force 'ON', 0xff00 to force 'OFF'\n");
                RxBuffer[1] += 128;
            }
            crcResult = CRCGen(RxBuffer, 2);
            RxBuffer[6] = (crcResult & 0xff);
            RxBuffer[7] = ((crcResult >> 8) & 0xff);
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func6(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int forced, address, crcflag;
    uint16_t crcResult;
    address = stick(frame[2], frame[3]);
    forced = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 1);

    RxBuffer[2] = frame[2];
    RxBuffer[3] = frame[3];
    RxBuffer[4] = frame[4];
    RxBuffer[5] = frame[5];
    if (crcflag == 0) {
        if ((address < 0) | (address > 9998)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            RxBuffer[1] = frame[1];
            holdingRegs[address] = forced;

            crcResult = CRCGen(RxBuffer, 2);
            RxBuffer[6] = (crcResult & 0xff);
            RxBuffer[7] = ((crcResult >> 8) & 0xff);
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func15(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 2);

    RxBuffer[2] = frame[2];
    RxBuffer[3] = frame[3];
    RxBuffer[4] = frame[4];
    RxBuffer[5] = frame[5];
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 2000) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            if (biggerMark8Coils(count) == frame[6]) {
                RxBuffer[1] = frame[1];
                //for (int j = 0; j < RxBuffer[2]; j++) {
                //    RxBuffer[3 + j] = 0x00;
                //}
                for (int i = 0; i < count; i++) {
                    coils[start + i] = ((frame[7 + (i / 8)] >> (i % 8)) & 0x01);
                }
                crcResult = CRCGen(RxBuffer, 2);
                RxBuffer[6] = (crcResult & 0xff);
                RxBuffer[7] = ((crcResult >> 8) & 0xff);
            }
            else {
                printf("Byte count doesn't match the quantity of coils, Something's fishy!\n");
                RxBuffer[1] = frame[1] + 128;
            }
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void func16(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int count, start, crcflag;
    uint16_t crcResult;
    start = stick(frame[2], frame[3]);
    count = stick(frame[4], frame[5]);

    crcflag = RxCRCcheck(frame, 2);

    RxBuffer[2] = frame[2];
    RxBuffer[3] = frame[3];
    RxBuffer[4] = frame[4];
    RxBuffer[5] = frame[5];
    if (crcflag == 0) {
        if ((start < 0) | (start > 9998) | (((start + count) - 1) > 9998) | (count > 125) | (count < 1)) {
            printf("Invalid memory access\n");
            RxBuffer[1] = frame[1] + 128;
        }
        else {
            if (count * 2 == frame[6]) {
                RxBuffer[1] = frame[1];
                //for (int j = 0; j < RxBuffer[2]; j++) {
                //    RxBuffer[3 + j] = 0x00;
                //}
                for (int i = 0; i < count; i++) {
                    holdingRegs[start + i] = stick(frame[7 + (i * 2)], frame[8 + (i * 2)]);
                }
                crcResult = CRCGen(RxBuffer, 2);
                RxBuffer[6] = (crcResult & 0xff);
                RxBuffer[7] = ((crcResult >> 8) & 0xff);
            }
            else {
                printf("Byte count doesn't match the quantity of Holding registers, Something's fishy!\n");
                RxBuffer[1] = frame[1] + 128;
            }
        }
    }
    else {
        printf("CRC Error occurred\n");
        RxBuffer[1] = frame[1] + 128;
    }
    if (RxBuffer[1] == frame[1] + 128) {
        crcResult = CRCGen(RxBuffer, 2);
        RxBuffer[6] = (crcResult & 0xff);
        RxBuffer[7] = ((crcResult >> 8) & 0xff);
    }
}

void call_functions(uint8_t frame[256], uint8_t RxBuffer[256]) {
    int address = frame[0];
    int func = frame[1];
    RxBuffer[0] = myAddress; // Is this correct?
    if ((address == myAddress) | (address == 0)) {
        switch (func)
        {
        case 1:
            if (address == 0) {
            printf("Broadcasting isn't supported for this function\n");
            RxBuffer[1] = frame[1] + 128;
            }
            else {
                func1(frame, RxBuffer);
            }
            break;
        case 2:
            if (address == 0) {
            printf("Broadcasting isn't supported for this function\n");
            RxBuffer[1] = frame[1] + 128;
            }
            else {
                func2(frame, RxBuffer);
            }
            break;
        case 3:
            if (address == 0) {
            printf("Broadcasting isn't supported for this function\n");
            RxBuffer[1] = frame[1] + 128;
            }
            else {
                func3(frame, RxBuffer);
            }
            break;
        case 4:
            if (address == 0) {
            printf("Broadcasting isn't supported for this function\n");
            RxBuffer[1] = frame[1] + 128;
            }
            else {
                func4(frame, RxBuffer);
            }
            break;
        case 5:
            func5(frame, RxBuffer);
            break;
        case 6:
            func6(frame, RxBuffer);
            break;
        case 15:
            func15(frame, RxBuffer);
            break;
        case 16:
            func16(frame, RxBuffer);
            break;
        default:
            printf("Invalid function used\n");
            RxBuffer[1] = frame[1] + 128;
            break;
        }
    }
    else {
        printf("Not my address!\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RxDone = 1;
	countinCycles3++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_AbortTransmit(&huart2);
	TxDone = 1;
	countinCycles4++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	countinCycles2++;
	timerFlag = 1;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
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
