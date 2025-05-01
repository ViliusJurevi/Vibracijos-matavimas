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
#include "lcd.h"
#include <stdio.h>
#include "string.h"
#include "Statechart.h"
#include "Statechart_required.h"
#include <math.h>

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TxBuffer[120];
uint8_t RxBuffer[20];


#define MPU_ADDR 0x68 << 1  // I2C address of the MPU-6050
#define AFS_SEL 2           // Accelerometer Configuration Settings   AFS_SEL=2, Full Scale Range = +/- 8 [g]
#define DLPF_SEL 0          // DLPF Configuration Settings  Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz

int16_t AcX, AcY, AcZ;            // Accelerometer values
long Cal_AcX, Cal_AcY, Cal_AcZ;   // Calibration values
float GAcX, GAcY, GAcZ;           // Gravity values
float Min_GAcX = 0, Max_GAcX = 0, PtoP_GAcX;
float Min_GAcY = 0, Max_GAcY = 0, PtoP_GAcY;
float Min_GAcZ = 0, Max_GAcZ = 0, PtoP_GAcZ;
float Grvt_unit;  // Gravity value unit
long prev_time;   // Period of calculation
int16_t AccelX;
int16_t AccelY;
int16_t AccelZ;
Statechart handle;
int sample_no = 0;

float sumSq_GAcX = 0;
float sumSq_GAcY = 0;
float sumSq_GAcZ = 0;


float RMS_GAcX;
float RMS_GAcY;
float RMS_GAcZ;

float RMS_GAcX_LCD;
float RMS_GAcY_LCD;
float RMS_GAcZ_LCD; 



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  statechart_raise_timer(&handle); //raise event TimerIntr in statechart
	statechart_send_Uart(&handle);
	statechart_display_Grvt(&handle);
//Enable key interrupts handling again

}


void LCD_Write4Bits(uint8_t data)
{
    HAL_GPIO_WritePin(D4_PORT, D4_PIN, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D5_PORT, D5_PIN, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D6_PORT, D6_PIN, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D7_PORT, D7_PIN, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_SendCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_RESET); // RS = 0 (komanda)
    HAL_GPIO_WritePin(RW_PORT, RW_PIN, GPIO_PIN_RESET);  // R/W = 0 (ra?ymas)
    
    LCD_Write4Bits(cmd >> 4);  // Auk?tesni 4 bitai
    LCD_Write4Bits(cmd & 0x0F); // ?emesni 4 bitai
    if (cmd < 4) HAL_Delay(2);  // Jei komanda buvo "Clear" ar "Home"
}

void LCD_SendData(uint8_t data)
{
    HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_SET);  // RS = 1 (duomenys)
    HAL_GPIO_WritePin(RW_PORT, RW_PIN, GPIO_PIN_RESET); // R/W = 0 (ra?ymas)
    
    LCD_Write4Bits(data >> 4);  // Auk?tesni 4 bitai
    LCD_Write4Bits(data & 0x0F); // ?emesni 4 bitai
    HAL_Delay(1);
}

void statechart_lCD_Init(Statechart* handle){
	HAL_Delay(50);
    HAL_GPIO_WritePin(RW_PORT, RW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_RESET);
    LCD_Write4Bits(0x03);
    HAL_Delay(5);
    LCD_Write4Bits(0x03);
    HAL_Delay(5);
    LCD_Write4Bits(0x03);
    HAL_Delay(1);
    LCD_Write4Bits(0x02);  // 4-bitu re?imas
    HAL_Delay(1);
    LCD_SendCommand(0x28);  // 4-bitu re?imas, 2 eilutes, 5x8 simboliai
    LCD_SendCommand(0x0C);  // Ijungti ekrana, i?jungti kursoriu
    LCD_SendCommand(0x06);  // Inkrementuoti kursori
    LCD_SendCommand(0x01);  // I?valyti ekrana
    HAL_Delay(2);
	
}
void LCD_Clear(void)
{
    LCD_SendCommand(0x01); // I?valyti ekrana
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_SendCommand(0x80 | (col + row_offsets[row]));
}

void LCD_SendString(char *str)
{
    while (*str) LCD_SendData(*str++);
}

void statechart_mPU6050_Init(Statechart* handle) {
    uint8_t data;

    // Wake up MPU6050
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);

    // Set Clock Source
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);

    // Set accelerometer range to +/- 2g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);

    // Set Digital Low Pass Filter to 260Hz
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1A, 1, &data, 1, HAL_MAX_DELAY);
}

void statechart_gravity_Range(Statechart* handle) {
    Grvt_unit = 4096.0;
}

void statechart_mPU6050_Read_Accel(Statechart* handle) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, 1, data, 6, HAL_MAX_DELAY);

    AcX = (int16_t)(data[0] << 8 | data[1]);
    AcY = (int16_t)(data[2] << 8 | data[3]);
    AcZ = (int16_t)(data[4] << 8 | data[5]);
}

void statechart_mPU6050_Calibrate(Statechart* handle) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < 2000; i++) {
        statechart_mPU6050_Read_Accel(handle);
        sumX += AcX;
        sumY += AcY;
        sumZ += AcZ;
        HAL_Delay(1);
    }

    Cal_AcX = sumX / 2000;
    Cal_AcY = sumY / 2000;
    Cal_AcZ = sumZ / 2000;
}

void statechart_calculate_Gravity(Statechart* handle) {
    // Iš akselerometro nuskaitytų verčių korekcija atimant kalibracijos metu gautus poslinkius
    int16_t adjX = AcX - Cal_AcX;
    int16_t adjY = AcY - Cal_AcY;
    int16_t adjZ = AcZ - Cal_AcZ;

    // Koreguotos vertės paverčiamos į gravitacijos vienetus (g)
    GAcX = (float)adjX / Grvt_unit;
    GAcY = (float)adjY / Grvt_unit;
    GAcZ = (float)adjZ / Grvt_unit;

    // X ašies min ir max reikšmių fiksavimas, naudojamas (peak-to-peak) skaičiavimui
    if (GAcX < Min_GAcX) Min_GAcX = GAcX;
    if (GAcX > Max_GAcX) Max_GAcX = GAcX;
    PtoP_GAcX = Max_GAcX - Min_GAcX;

    // Y ašies min ir max reikšmių fiksavimas
    if (GAcY < Min_GAcY) Min_GAcY = GAcY;
    if (GAcY > Max_GAcY) Max_GAcY = GAcY;
    PtoP_GAcY = Max_GAcY - Min_GAcY;

    // Z ašies min ir max reikšmių fiksavimas
    if (GAcZ < Min_GAcZ) Min_GAcZ = GAcZ;
    if (GAcZ > Max_GAcZ) Max_GAcZ = GAcZ;
    PtoP_GAcZ = Max_GAcZ - Min_GAcZ;
}

void statechart_display_Grvt(Statechart* handle) {
  

        // Kaupti kvadratines reiksmes
        sumSq_GAcX += GAcX * GAcX;
        sumSq_GAcY += GAcY * GAcY;
        sumSq_GAcZ += GAcZ * GAcZ;
			
			  RMS_GAcX_LCD = sqrtf(sumSq_GAcX / sample_no);
        RMS_GAcY_LCD = sqrtf(sumSq_GAcY / sample_no);
        RMS_GAcZ_LCD = sqrtf(sumSq_GAcZ / sample_no);
        sample_no++;

       
    }


void statechart_display_LCD(Statechart* handle){
    char buf[40];

    LCD_Clear();

    LCD_SetCursor(0, 0);
    sprintf(buf, "Ax=%-6.2f", RMS_GAcX_LCD);
    LCD_SendString(buf);

    LCD_SetCursor(1, 0);
    sprintf(buf, "Ay=%-6.2f", RMS_GAcY_LCD);
    LCD_SendString(buf);

    LCD_SetCursor(2, 0);
    sprintf(buf, "Az=%-6.2f", RMS_GAcZ_LCD);
    LCD_SendString(buf);

    
}



void statechart_send_Uart(Statechart* handle) {
  

    char uart_buffer[100];
    sprintf(uart_buffer, "X: %.2f, Y: %.2f, Z: %.2f\r\n", RMS_GAcX, RMS_GAcY, RMS_GAcZ);

    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

void statechart_rMS(Statechart* handle){
	
    RMS_GAcX = fabsf(GAcX); // nes viena imtis, tai tiesiog modulis
	
    RMS_GAcY = fabsf(GAcY);
	
    RMS_GAcZ = fabsf(GAcZ);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start_IT(&htim6);

    
		

   
/* MCU Configuration */
    
    
    
    /* Welcome message */
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("MPU6050 Accel");
    LCD_SetCursor(1, 0);
    LCD_SendString("Starting...");
    HAL_Delay(3000);
    
    statechart_lCD_Init(&handle);               // LCD startas
		
    statechart_mPU6050_Init(&handle);           // MPU startas
		
    statechart_gravity_Range(&handle);          // nustatom G vienetus
		
    statechart_mPU6050_Calibrate(&handle);      // kalibracija
    
    
    
    
   

   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
		
		   
		    


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		statechart_mPU6050_Read_Accel(&handle);
    statechart_calculate_Gravity(&handle);
    statechart_rMS(&handle);
		
		if (sample_no >= 30) {
        statechart_display_LCD(&handle); // Atvaizduojame LCD
        sumSq_GAcX = 0;
        sumSq_GAcY = 0;
        sumSq_GAcZ = 0;
        sample_no = 0;
      }
 
		
		  
			
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000000;
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

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 2000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 105;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
