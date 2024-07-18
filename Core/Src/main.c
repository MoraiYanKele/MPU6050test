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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t MPUdata[14];             // 接收mpu6050的数据
float accel_x, accel_y, accel_z; // 加速度计数据（米/秒^2）
float gyro_x, gyro_y, gyro_z;    // 陀螺仪数据（度每秒）
uint8_t uartTxBuffer[17];        // 串口发送数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

void SendData(float pitch, float roll, float yaw)
{
  
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
  uint16_t u16_pitch = (uint16_t)(pitch * 100);
  uint16_t u16_roll = (uint16_t)(roll * 100);
  uint16_t u16_yaw = (uint16_t)(yaw * 100);
  
  
	int _cnt=0;
	uartTxBuffer[_cnt++] = 0xAA;//帧头
	uartTxBuffer[_cnt++] = 0xFF;//目标地址

	uartTxBuffer[_cnt++] = 0X03;//功能码
	uartTxBuffer[_cnt++] = 0x07;//数据长度

	uartTxBuffer[_cnt++] = u16_pitch;//数据内容,小段模式，低位在前
	uartTxBuffer[_cnt++] = u16_pitch >> 8;//需要将字节进行拆分，调用上面的宏定义即可。
	uartTxBuffer[_cnt++] = u16_roll;
	uartTxBuffer[_cnt++] = u16_roll >> 8;	
	uartTxBuffer[_cnt++] = u16_yaw;
	uartTxBuffer[_cnt++] = u16_yaw >> 8;

	uartTxBuffer[_cnt++] = 0;

	//SC和AC的校验直接抄最上面上面简介的即可
	for (int i = 0; i < uartTxBuffer[3] + 4; i++) 
	{
		sumcheck += uartTxBuffer[i];
    addcheck += sumcheck;
		
	} 
	uartTxBuffer[_cnt++] = sumcheck;	
	uartTxBuffer[_cnt++] = addcheck;	


  HAL_UART_Transmit(&huart1, uartTxBuffer, _cnt, 0xffff);

}
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
  int result;
  float pitch = 0, roll = 0, yaw = 0; 
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  printf("ready\n");
  result = MPU6050_DMP_Init();
  if (result != 0)
  {
    printf("error1 %d\n", result);
 
    while (result)
    {
      HAL_Delay(500);
      result = MPU6050_DMP_Init();
      printf("error2 %d\n", result);
    }
    
  }
  else
  {
    printf("success");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    if (MPU6050_DMP_GetData(&pitch, &roll, &yaw) == 0)
    {
      // printf("%f, %f, %f\n", pitch, roll, yaw);
      SendData(pitch, roll, yaw);
    }
    HAL_Delay(10);

    /* USER CODE END WHILE */

    // 读取加速度计数据
    // accel_x = ReadAccel_x(MPUdata);
    // accel_y = ReadAccel_y(MPUdata);
    // accel_z = ReadAccel_z(MPUdata);

    // // 读取陀螺仪数据
    // gyro_x = ReadGyro_x(MPUdata);
    // gyro_y = ReadGyro_y(MPUdata);
    // gyro_z = ReadGyro_z(MPUdata);

    // printf("%f, %f, %f, %f, %f, %f\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    // HAL_Delay(10);


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
