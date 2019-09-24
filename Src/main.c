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
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "input.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define division 0.046019 // linear mm per encoder count: pi*60mm/4096
#define PixelCountX 108.7 // Pixels per encoder count (X Axis)
#define xRpmDAC  1.365    // DAC per RPM X Axis
#define yRpmDAC  0.683    // DAC per RPM Y Axis
#define Yenable SET       // Enable Y Axis
#define Ydisable RESET    // Disable Y Axis
#define Xenable RESET     // Enable X Axis
#define Xdisable SET      // Disable X Axis
#define CCW RESET         // X Axis CCW Rotation
#define CW SET            // X Axis CW Rotation


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag = 0;    // Timer interrupt flag
volatile uint16_t posX = 0;   // combine all four POS in array
volatile uint16_t posY = 0;
volatile uint16_t posXNew;
volatile uint16_t posYNew;
volatile uint16_t malletX = 0;  // combine mallet in array
volatile uint16_t malletY = 0;
volatile float rpmX = 0.0;      // combine rpm in array
volatile float rpmY = 0.0;
volatile uint16_t puck[2];
volatile uint16_t msg[11];
volatile uint8_t testPosX = 75;
int32_t* rpm2DACX;
uint16_t testMAG;
uint8_t cycleNum;
uint8_t discardCycles;
int16_t rpmTarget;
int16_t rpmTargetX;
int16_t rpmTargetY;
volatile uint32_t count = 0;
const uint8_t puckPositionArray[5] = {50,75,140,30,95};
uint8_t index_point = 0;
uint8_t sysID[21];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setXSpeed (int16_t rpm);
void setYSpeed (int16_t rpm);

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
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Start UART DMA
  HAL_UART_Transmit_DMA(&huart2, sysID, 21);


  // SPI init and start DMA
  HAL_SPI_Init(&hspi2);
  HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)puck, 2);  // 2 16-bit words, X and Y position


  // Start timer interrupts
  HAL_TIM_Base_Start_IT(&htim2);                    // Sampling Period
  HAL_TIM_Base_Start_IT(&htim3);                    // X Axis Emergency Stop
  HAL_TIM_Base_Start_IT(&htim4);                    // Y Axis Emergency Stop

  // Start Encoders
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   // X Axis
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   // Y Axis

  // Start and Initialise DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);              // X Axis
  setXSpeed(0);     // Set speed to Zero

  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);              // Y Axis
  setYSpeed(0);  // Set Speed to Zero

  // // Enable Motors
  // if (TIM3->CNT <= htim3.Init.Period) {               // Check X Axis
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);          // LED 5 off
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);        // Enable X Axis
  // }
  //
  // if (TIM4->CNT <= htim4.Init.Period) {               // Check Y Axis
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);         // LED 6 off
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);        // Enable Y Axis
  // }


  //  Test code for comissioning
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, Xdisable);   // X Axis enable/disable
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, Yenable);   // Y Axis enable/disable
  // Set Speed, positive for forward, negative for reverse
  // setXSpeed(60);
  // setYSpeed(20);


  // Center encoder count for testing, remove later
  (TIM3->CNT) = 4095;
  (TIM4->CNT) = 35000;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Master timer elapsed
    if (flag == 1) {

      // Test parameters
      testMAG = 3000;     // Max RPM
      cycleNum = 15;      // Number of cycles to record
      discardCycles = 3;  // Number of cycles to discard at beginning

      // Send input to motor
      if (count < (discardCycles+cycleNum)*10000) {
        rpmTargetX = (int)(testMAG*sineX[count%10000]);
        rpmTargetY = (int)(testMAG*sineY[count%10000]);
        setXSpeed(rpmTargetX);   // X Axis
        setYSpeed(rpmTargetY);   // Y Axis
     }

     // Stop input after determined number of cycles
     else{
       rpmTarget = 0;
       setXSpeed(rpmTarget);   // X Axis
       setYSpeed(rpmTarget);   // Y Axis
     }


      // Determine Position
      malletX = (TIM3->CNT)/PixelCountX;
      malletY = (TIM4->CNT)/PixelCountX;

      // Determine RPM
      if (count%100 == 0) {
        posXNew = (TIM3->CNT);
        posYNew = (TIM4->CNT);
        rpmX = (((posXNew-posX)/4095.0)*6000);
        rpmY = (((posYNew-posY)/4095.0)*6000);
        posX = posXNew;
        posY = posYNew;

        // Transmit data to UART
        if ((count >= discardCycles*10000) && count < (discardCycles+cycleNum)*10000+100) {
          // sprintf(sysID, "%05i, %05i, %06i\n", count/10, rpmTargetY, (int)rpmX);  // X Axis
          sprintf(sysID, "%05i, %05i, %06i\n", count/10, rpmTargetY, (int)rpmY);  // Y Axis

        }

        flag = 0;
    }
  }
  //
  //   // // test position array increment every 2 seconds
  //   // if (count%2000 == 0) {
  //   //   testPosX = puckPositionArray[index_point%5];
  //   //   index++;
  //   // }
  //   //
    // Move X Axis to meet puck

    // // HAL_UART_Transmit(&huart2, msg, sizeof(msg), 10);
    // if (puck[1] >= 8 && puck[1] <= 143) {
    //   if ((uint16_t)fabs(puck[1]-malletX) >= 1) {
    //     setXSpeed((puck[1]-malletX)*25);
    //   }
    // }
    // else{
    //   setXSpeed(0);
    // }
    // sprintf(sysID, "%05d, %05d\n", puck[1], puck[0]);
    // HAL_UART_Transmit(&huart2, sysID, sizeof(sysID), 10);


    // // Transmit X Position, Y position to UART
    // uint8_t posMsg[11];
    // sprintf(posMsg, "%03i, %03i\n", puck[0], puck[1]);
    // HAL_UART_Transmit(&huart2, posMsg, sizeof(posMsg), 10);
    // HAL_Delay(250);
    //

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Set X Axis speed and direction
void setXSpeed (int16_t rpm)
{
  uint32_t spd = (uint32_t)fabs(rpm*1.365);
  if (rpm==0) {
    spd = 0;
  }
  if (rpm < 0) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, CCW);         // X Axis rotation
  }
  else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, CW);         // X Axis rotation
  }
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, spd);
}

// Set Y Axis speed and direction
void setYSpeed (int16_t rpm)
{
  uint32_t spd = 2047+(rpm*0.68267);
  if (rpm==0) {
    spd = 2047;
  }
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, spd);
}

// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//   sprintf(sysID, "%05d, %05d\n", puck[1], puck[0]);// HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
// }

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
