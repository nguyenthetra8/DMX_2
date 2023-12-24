/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "DMX512.h"
#include "usbd_cdc_if.h"
#include "74hc595.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum e_leddigit { P = 10,C = 11,S = 12,D = 13}led7_e;
extern uint8_t dmxData[513];
#define MAX_VALUES 512

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char buffer[1024];
FATFS fs;
FIL fil;
FRESULT fresult;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t Counter = 0;

int		ma7doan[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0xff,0xcf,0x9e,0x5b,0xc0};
unsigned int t,led1,led2;
extern uint8_t USB_BUFF[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void xuat_595(unsigned int  tt);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void ScanLed(uint8_t num,uint8_t chanel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_uart(char* c){
	uint8_t len = strlen(c);	
	HAL_UART_Transmit(&huart2,(uint8_t*)c,4096,100);

}
int bufsize(char* buf){
	int i = 0;
	while (*buf++ != '\0') i++;
	return i;
}
void bufclear(void){
	for(int i=0;i<1024;i++){
		buffer[i] = '\0';
	}
}
void shift_led(uint8_t data){
	uint8_t temp;
	HAL_GPIO_WritePin(GPIOB,LAD_LED_Pin,0);
	for(int i =0;i<8;i++){
		temp = data&(0x80>>i);
		if(temp == 0)
			HAL_GPIO_WritePin(GPIOB,DATA_LED_Pin,0);
		else
			HAL_GPIO_WritePin(GPIOB,DATA_LED_Pin,1);
		HAL_GPIO_WritePin(GPIOB,CLK_LED_Pin,1);
		HAL_GPIO_WritePin(GPIOB,CLK_LED_Pin,0);
	}
	HAL_GPIO_WritePin(GPIOB,LAD_LED_Pin,1);
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
		char line[300]; // M?ng kí t? d? luu t?ng dòng
    char data[512]={0};
		char add[512]= {0};
    char* token;
		char* sub_token;
    uint16_t hold;
		int nr_of_slot;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) send_uart ("ERROR!!! in mounting SD CARD...\n\n");
	else send_uart("SD CARD mounted successfully...\n\n");
		fresult = f_open(&fil, "show2.txt", FA_READ);
	if (fresult == FR_OK)send_uart ("show1.txt is open and the data is shown below\n");
	
	/* Read data from the file
	 * Please see the function details for the arguments 
	*/
	while (f_gets(line, sizeof(line), &fil)) { // Ð?c t?ng dòng t? file cho d?n khi h?t file
      
			int i1 =0; 
			token = strtok(line, ",");// G?i hàm strtok() l?n d?u tiên v?i chu?i và kí t? phân cách là d?u ph?y
			while((token != NULL))  // L?p l?i cho d?n khi không còn ph?n nào n?a
			{		
				if(i1 == 0)						//so dau tien lay so luong dia chi
					nr_of_slot = atoi(token);
				if((i1/2)<nr_of_slot ){
					if((i1 % 2) == 0)		//so o vi chi chan luu vao data
						data[i1 / 2] = atoi(token);
					else							// so o vi tri le luu vao add
						add[i1 / 2] = atoi(token);
				}
				else								// so cuoi dong la holdtime
					hold = atoi(token);
				i1++;
				
			}
			for(int t =0;t<nr_of_slot;t++){
				
				dmxData[add[t]]= data[t];
			}
			DMX_send_ms(hold,(uint8_t*)data);
			nr_of_slot = 0;
			i1 =0;
			token = strtok(NULL, ","); // G?i hàm strtok() ti?p theo v?i tham s? d?u tiên là NULL
	}
//	/* Close file */
	f_close(&fil);
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim2.Init.Prescaler = 3600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
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
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LAD_LED_Pin|CLK_LED_Pin|D1_Pin|D2_Pin
                          |D3_Pin|D4_Pin|DATA_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LAD_LED_Pin CLK_LED_Pin D1_Pin D2_Pin
                           D3_Pin D4_Pin DATA_LED_Pin */
  GPIO_InitStruct.Pin = LAD_LED_Pin|CLK_LED_Pin|D1_Pin|D2_Pin
                          |D3_Pin|D4_Pin|DATA_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void delay_us(uint32_t us){
	 __HAL_TIM_SET_COUNTER(&htim4,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}
void delay_ms(uint32_t ms){
	 __HAL_TIM_SET_COUNTER(&htim4,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim4) < 1000*ms);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == htim2.Instance)
 {
   Counter++;
 }
}

void xuat_595(unsigned int  tt)
{
	unsigned int	tam,i;
	tam=tt;
	for(i=0;i<8;i++)			// 100 10100  1000 0000
	{
		if((tam&0x80)==0x80)		HAL_GPIO_WritePin(GPIOB,DATA_LED_Pin,1);
		else										HAL_GPIO_WritePin(GPIOB,DATA_LED_Pin,0);
		HAL_GPIO_WritePin(CLK_LED_GPIO_Port,CLK_LED_Pin,1);			HAL_Delay(1);
		HAL_GPIO_WritePin(CLK_LED_GPIO_Port,CLK_LED_Pin,0);			HAL_Delay(1);
		tam=tam<<1;
	}
}

void ScanLed(uint8_t num,uint8_t chanel){   // chanel (0: SD; 1: PC )
	uint8_t d1,d2,d3,d4;
	unsigned int t1,t2;
	t1 = num/10;
	t2 = num%10;
	switch(chanel){
		case 0:
			if(Counter == 10 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,0);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[S],HIGH);
			}
			if(Counter == 20 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,0);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[D],HIGH);
			}
			if(Counter == 30 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,0);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[t1],HIGH);
			}
			if(Counter == 40 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,0);
				ShiftRegister74HC595_setPin(ma7doan[t2],HIGH);
			}
			break;
		case 1:
			if(Counter == 10 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,0);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[P],HIGH);
			}
			if(Counter == 20 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,0);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[C],HIGH);
			}
			if(Counter == 30 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,0);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,1);
				ShiftRegister74HC595_setPin(ma7doan[t1],HIGH);
			}
			if(Counter == 40 ){
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,1);
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D3_GPIO_Port,D2_Pin,1);
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,0);
				ShiftRegister74HC595_setPin(ma7doan[t2],HIGH);
			}
			break;
	}
	
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
// if(huart->Instance == huart2.Instance)
// {
//   HAL_UART_Receive_IT(&huart2, buffer, 200);
// }
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
