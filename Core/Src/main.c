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
#include "Thruster.h"
#include "hydrolib_ring_queue.h"
#include "ssd1306.h"
/* USER CODE END Includes */
#include "stdio.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t buffer_RX[80];
uint8_t buffer_ring[128];
hydrolib_RingQueue ringQueue;
int16_t buffer_index;
uint16_t data[12];
float adcVoltage = 0.0;
uint16_t startIdx = 0;
char voltageStr[12];
uint8_t adres_i2c = 1;
int flt = 123.45;
const unsigned char ahegao_map[] = {

		0x00, 0x00, 0x00, 0x1f, 0xff, 0xf0, 0xe0, 0x00, 0x20, 0x00, 0x04, 0x40, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x54, 0x00, 0x11, 0x00, 0x02, 0x20, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0x57, 0xc2, 0xb4, 0x00, 0x91, 0x00, 0x01, 0x1f, 0xf8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x10, 0x00, 0x01, 0xad, 0x00, 0x98, 0x80, 0x00, 0x95, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x8a, 0x01, 0x38, 0x88, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x0c, 0x10, 0x8a, 0x01, 0x68, 0x84, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x08, 0x21, 0x84, 0x00, 0x40, 0xc4, 0x00, 0x03, 0x50, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x8c, 0x02, 0xc4, 0x82, 0x00, 0x13, 0xf8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x12, 0x41, 0x54, 0x03, 0x80, 0xa2, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x24, 0x43, 0x34, 0x03, 0x02, 0x41, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x48, 0x07, 0x3c, 0x07, 0x02, 0x51, 0x04, 0x04, 0xf8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x40, 0x87, 0x5c, 0x06, 0xc1, 0x48, 0x82, 0x04, 0x80, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x90, 0x8d, 0x58, 0x0c, 0x10, 0x40, 0x80, 0x02, 0x78, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x80, 0x8d, 0x5c, 0x08, 0x04, 0xa2, 0x41, 0x00, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0x20, 0x09, 0x18, 0x08, 0x01, 0x60, 0x60, 0x81, 0x20, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1f, 0x21, 0x09, 0x5c, 0x10, 0x37, 0xf1, 0x20, 0x40, 0x10, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x41, 0x11, 0xbc, 0x10, 0x00, 0x7e, 0xa8, 0x60, 0xc0, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x02, 0xc1, 0x11, 0x48, 0x10, 0x7f, 0xf0, 0xf2, 0x28, 0x68, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x02, 0x41, 0x11, 0x14, 0x00, 0x7f, 0xf8, 0x19, 0x10, 0x50, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1d, 0xc0, 0x83, 0xf0, 0x20, 0xff, 0xff, 0x18, 0x8a, 0x28, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x04, 0x80, 0x27, 0x12, 0x20, 0xfd, 0xff, 0x94, 0x45, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x06, 0x80, 0xa9, 0xf2, 0x20, 0x7d, 0xe7, 0x92, 0x10, 0x90, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x0a, 0x80, 0xb7, 0xfa, 0x00, 0x3f, 0x43, 0x92, 0x06, 0x48, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x08, 0x80, 0xe7, 0xf9, 0x40, 0x16, 0x41, 0x94, 0x01, 0x30, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1c, 0x80, 0xbf, 0x79, 0x40, 0x12, 0x82, 0x20, 0x00, 0x98, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1e, 0x80, 0x3f, 0x79, 0x40, 0x0a, 0x00, 0x28, 0x20, 0x38, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1a, 0x10, 0x7b, 0xd0, 0xff, 0x80, 0x00, 0x20, 0x00, 0x18, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1e, 0x90, 0x5b, 0xe1, 0xef, 0xcf, 0xf7, 0xb0, 0x60, 0x28, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x19, 0x10, 0x32, 0x22, 0xff, 0xbf, 0xfa, 0x70, 0x40, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1e, 0x92, 0x61, 0x44, 0x1f, 0x5f, 0xfc, 0x70, 0xa0, 0x10, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x10, 0x10, 0x20, 0xe4, 0x06, 0xbe, 0xe0, 0xf0, 0x28, 0x90, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x17, 0x91, 0x70, 0x38, 0x00, 0x50, 0x40, 0xe1, 0x28, 0x90, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x04, 0x91, 0x53, 0xf2, 0x00, 0x00, 0x01, 0xa2, 0x28, 0x88, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x04, 0xb1, 0xaf, 0xe0, 0x00, 0x00, 0x01, 0xa0, 0x28, 0x98, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x02, 0xb1, 0x2f, 0xc0, 0x00, 0x00, 0x02, 0x24, 0x28, 0x58, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x0f, 0xb1, 0x5f, 0x00, 0x00, 0x00, 0x04, 0x48, 0x29, 0x58, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x08, 0x59, 0x39, 0x00, 0x0e, 0x00, 0x08, 0x40, 0x09, 0x58, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x08, 0x51, 0x28, 0x00, 0x69, 0xc0, 0x00, 0x10, 0x29, 0x58, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1f, 0xdd, 0x38, 0x00, 0xbf, 0xe8, 0x00, 0xa0, 0x49, 0x58, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x10, 0xd5, 0x0c, 0x00, 0xff, 0xf8, 0x00, 0x60, 0x49, 0xd0, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1f, 0xfd, 0x14, 0x01, 0xff, 0xf8, 0x01, 0x48, 0x4b, 0xd0, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x3d, 0x12, 0x03, 0xff, 0xfc, 0x01, 0x28, 0x49, 0xd0, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x0a, 0xb6, 0x89, 0x03, 0xff, 0xf8, 0x02, 0x08, 0x8b, 0xd8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1f, 0xf2, 0x88, 0x03, 0xff, 0xfc, 0x02, 0x48, 0x93, 0xf8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x33, 0x88, 0x01, 0xff, 0xf8, 0x04, 0x48, 0x93, 0xd8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x0a, 0x86, 0x00, 0xff, 0xf8, 0x00, 0xc8, 0x95, 0xf8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x2b, 0xc6, 0x00, 0xff, 0xf8, 0x01, 0x51, 0x16, 0xf0, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x25, 0x26, 0x80, 0x7f, 0xf8, 0x02, 0x51, 0x17, 0x78, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x09, 0x22, 0xc0, 0x7f, 0xf0, 0x08, 0x51, 0x2b, 0xe8, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x09, 0x25, 0x60, 0x7f, 0xf0, 0x00, 0x50, 0x28, 0x78, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x15, 0x50, 0xb3, 0x70, 0x7f, 0xe0, 0x20, 0x90, 0x28, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x90, 0xb2, 0xa8, 0x7f, 0xc0, 0x40, 0xb0, 0x20, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0xe0, 0xb0, 0x24, 0x3f, 0xc0, 0x80, 0xb0, 0x10, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x1e, 0xa0, 0x28, 0x32, 0x3f, 0x83, 0x00, 0xb0, 0x50, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x20, 0xaa, 0x11, 0x1f, 0x0e, 0x00, 0xb0, 0x40, 0x40, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x40, 0x6a, 0x10, 0x9f, 0x1e, 0x00, 0x90, 0x50, 0x48, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0x40, 0x24, 0x0a, 0x6e, 0x3c, 0x00, 0x50, 0x60, 0x10, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x40, 0x65, 0x0b, 0x3f, 0xf8, 0x00, 0xa8, 0x40, 0x60, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x80, 0x65, 0x07, 0xb0, 0x70, 0x00, 0x48, 0x21, 0x80, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x80, 0x61, 0x85, 0x90, 0x20, 0x00, 0x48, 0x24, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0x00, 0x62, 0x81, 0xd8, 0x10, 0x00, 0xc8, 0x38, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x01, 0x00, 0x62, 0xc1, 0x5c, 0x18, 0x00, 0x48, 0x20, 0x00, 0x00, 0x00, 0x00,
		};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void ProcessUARTData(void);
void Configure_ADC_Channel_5(void);


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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	Thruster_Init();
	hydrolib_RingQueue_Init(&ringQueue, buffer_ring, sizeof(buffer_ring));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_RX, 80);
	Configure_ADC_Channel_5();
	adres_i2c = SSD1306_Init();
	SSD1306_GotoXY (34,0); // goto 10, 10
	SSD1306_Puts("SAUVC25", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY (10,45);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_UART_Receive(&huart1, buffer_RX, 52, 100);
//		for (uint8_t i = 0; i < 28; i++) {   // Searching for the packet header
//		    if ((buffer_RX[i] == 0xff) && (buffer_RX[i + 1] == 0xfd)) {
//		        buffer_index = i;
//		    }
//		}
//
//		data[0] = ((buffer_RX[3+ buffer_index] << 8) | (buffer_RX[2 + buffer_index] & 0xFF));
//		data[1] = ((buffer_RX[5+ buffer_index] << 8) | (buffer_RX[4 + buffer_index] & 0xFF));
//		data[2] = ((buffer_RX[7 + buffer_index] << 8) | (buffer_RX[6+ buffer_index] & 0xFF));
//		data[3] = ((buffer_RX[9 + buffer_index] << 8) | (buffer_RX[8+ buffer_index] & 0xFF));
//		data[4] = ((buffer_RX[11 + buffer_index] << 8) | (buffer_RX[10+ buffer_index] & 0xFF));
//		data[5] = ((buffer_RX[13 + buffer_index] << 8) | (buffer_RX[12+ buffer_index] & 0xFF));
//		data[6] = ((buffer_RX[15 + buffer_index] << 8) | (buffer_RX[14+ buffer_index] & 0xFF));
//		data[7] = ((buffer_RX[17 + buffer_index] << 8) | (buffer_RX[16+ buffer_index] & 0xFF));
//		data[8] = ((buffer_RX[19 + buffer_index] << 8) | (buffer_RX[18+ buffer_index] & 0xFF));
//		data[9] = ((buffer_RX[21 + buffer_index] << 8) | (buffer_RX[20+ buffer_index] & 0xFF));
//		data[10] = ((buffer_RX[23 + buffer_index] << 8) | (buffer_RX[22+ buffer_index] & 0xFF));
//		data[11] = ((buffer_RX[25 + buffer_index] << 8) | (buffer_RX[24+ buffer_index] & 0xFF));
//
//		Thruster_Set_Speed(data);
//
		sprintf(voltageStr, "V_akb = %.2f", adcVoltage); // convert float to string
		SSD1306_GotoXY (10,45);// goto 10, 10
		SSD1306_Puts(voltageStr, &Font_7x10, 1);// print string
		SSD1306_UpdateScreen();// update screen
		ProcessUARTData();// Process the received data

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) { // Обработчик прерывания по приему данных по UART

	if (huart->Instance == USART1) { // Проверяем, что источник прерывания - UART1
		hydrolib_RingQueue_Push(&ringQueue, buffer_RX, Size); // Записываем принятые данные в кольцевой буфер
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_RX, sizeof(buffer_RX)); // Переводим UART в режим ожидания приема данных
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { // Обработчик прерывания по окончанию преобразования АЦП
	if (hadc->Instance == ADC1) {// Проверяем, что источник прерывания - АЦП1
		uint32_t adcValue = HAL_ADC_GetValue(hadc); // Считываем результат преобразования
		adcVoltage = (adcValue * 3.3*4.85) / 4095.0; //	Вычисляем напряжение на входе АЦП и умножаем на коэффициент деления резистивного делителя
		HAL_ADC_Start_IT(&hadc1); // Запускаем новое преобразование
	}
}

void ProcessUARTData(void) {

	for (uint8_t i = ringQueue.head; i < 12; i++) {	// Поиск заголовка пакета
		if ((hydrolib_RingQueue_ReadByte(&ringQueue, i) == 0xFF)
				&& (hydrolib_RingQueue_ReadByte(&ringQueue, i) == 0xFd)) {
			startIdx = i;// Запоминаем индекс начала пакета
			break;
		}
	}

	if (hydrolib_RingQueue_GetLength(&ringQueue) >= 26) {
		hydrolib_RingQueue_Drop(&ringQueue, startIdx + 2);// Удаляем заголовок пакета и все данные до него
		hydrolib_RingQueue_Pull(&ringQueue, data, 12);// Считываем данные пакета
		Thruster_Set_Speed(data);	// передаем сигнал на таймеры двигателями

	}
}

void Configure_ADC_Channel_5(void) { // Настройка канала АЦП
	ADC_ChannelConfTypeDef sConfig = { 0 }; // Структура для настройки канала АЦП

	// Configure ADC channel 5
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	// Start ADC in interrupt mode
	if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
		Error_Handler();
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
	while (1) {
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
