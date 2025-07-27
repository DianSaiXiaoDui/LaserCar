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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "CCR_PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint16_t startAngleX = 90;
uint16_t startAngleY = 70;

uint16_t currentAngleX;
uint16_t currentAngleY;

int16_t currentCCRX;
int16_t currentCCRY;

int32_t maxCCRX = 900;
int32_t minCCRX = 600;
int32_t maxCCRY = 900;
int32_t minCCRY = 600;


uint8_t Serial_RxBuffer1[128];
uint8_t RxIndex1 = 0;
uint8_t RxFlag1 = 0;
uint8_t ByteRcv1;

uint8_t Serial_RxBuffer2[128];
uint8_t RxIndex2 = 0;
uint8_t RxFlag2 = 0;
uint8_t ByteRcv2;

uint8_t Serial_RxBuffer4[128];
uint8_t RxIndex4 = 0;
uint8_t RxFlag4 = 0;
uint8_t ByteRcv4;

int16_t PID_delta_CCRX=0;
int16_t PID_delta_CCRY=0;
uint8_t PIDflag = 0;

float targetX;
float currentX;
float targetY;
float currentY;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   /* if (huart == &hlpuart1)
    {
        uint8_t receivedChar1 = ByteRcv1;
        if (receivedChar1 == '\r')
		{
			// 忽略\r字符
		}
        // 检测测消息结束符（换行符）hlpuart1
        else if (receivedChar1 == '\n')
        {
            Serial_RxBuffer1[RxIndex1] = '\0'; // 添加字符串终止符
            RxFlag1 = 1;
            RxIndex1 = 0;
        }
        else
        {
            Serial_RxBuffer1[RxIndex1++] = receivedChar1;
            if (RxIndex1 >= sizeof(Serial_RxBuffer1) - 1)
            {
                RxIndex1 = 0; // 防止缓冲区溢出
            }
        }
        // 重新启用接收中断
        HAL_UART_Receive_IT(&hlpuart1, &ByteRcv1, 1);
    }


    if (huart == &huart2)
	{
		uint8_t receivedChar2 = ByteRcv2;
		if (receivedChar2 == '\r')
		{
			// 忽略\r字符
		}
		else if (receivedChar2 == '\n')
		{
			Serial_RxBuffer2[RxIndex2] = '\0';
			RxFlag2 = 1;
			RxIndex2 = 0;
		}
		else
		{
			Serial_RxBuffer2[RxIndex2++] = receivedChar2;
			if (RxIndex2 >= sizeof(Serial_RxBuffer2) - 1)
			{
				RxIndex2 = 0;
			}
		}
		HAL_UART_Receive_IT(&huart2, &ByteRcv2, 1);
	}
*/

    if (huart == &huart4)
    {
		uint8_t receivedChar4 = ByteRcv4;
		if (receivedChar4 == '\r')
		{
			// 忽略\r字符
		}
		else if (receivedChar4 == '\n')
		{
			Serial_RxBuffer4[RxIndex4] = '\0';
			RxFlag4 = 1;
			RxIndex4 = 0;
		}
		else
		{
			Serial_RxBuffer4[RxIndex4++] = receivedChar4;
			if (RxIndex4 >= sizeof(Serial_RxBuffer4) - 1)
			{
				RxIndex4 = 0;
			}
		}
		HAL_UART_Receive_IT(&huart4, &ByteRcv4, 1);
    }
}

uint16_t Angle2CCR(uint16_t MaxAngle,uint16_t Angle)
{

   return Angle*1000/MaxAngle+250;
}

/*解析并执行串口命令*/
void ParseAndExecuteCommand(const char* buffer) {
	if (strncmp(buffer,"detect",6) == 0)
	{
		if (sscanf(buffer + 6, " targetX: %f currentX: %f targetY: %f currentY: %f", &targetX,&currentX,&targetY,&currentY) == 4)
		{
			CCRX_PID.Target = targetX;
			CCRX_PID.Current= currentX;
			CCRY_PID.Target = targetY;
			CCRY_PID.Current= currentY;
			PIDflag = 1;
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Angle2CCR(270,startAngleX));
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Angle2CCR(180,startAngleY));
  currentCCRX = Angle2CCR(270,startAngleX);
  currentCCRY = Angle2CCR(180,startAngleY);

  //HAL_UART_Receive_IT(&hlpuart1, &ByteRcv1, 1);
  //HAL_UART_Receive_IT(&huart2, &ByteRcv2, 1);
  HAL_UART_Receive_IT(&huart4, &ByteRcv4, 1);


  CCRX_PID_Init(0.2,0,0,0,0,0,0.2,0,0,15,0,0,0,0,55,-55,0);
  CCRY_PID_Init(0.15,0,0,0,0,0,0.2,0,0,15,0,0,0,0,55,-55,0);
  //HAL_UART_Transmit(&huart4,(uint8_t*)"Successfully initialized", 24, HAL_MAX_DELAY);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (RxFlag1)
	{
		ParseAndExecuteCommand((char*)Serial_RxBuffer1);
		//HAL_UART_Transmit(&huart1,(uint8_t*)Serial_RxBuffer1, strlen(Serial_RxBuffer1), HAL_MAX_DELAY);
		RxFlag1 = 0; // 处理完成
	}

	if(RxFlag2)
	{
		//HAL_UART_Transmit(&huart2,(uint8_t*)Serial_RxBuffer2, strlen(Serial_RxBuffer2), HAL_MAX_DELAY);
		ParseAndExecuteCommand((char*)Serial_RxBuffer2);
	    RxFlag2 = 0;
	}

	if(RxFlag4)
	{
		//HAL_UART_Transmit(&huart2,(uint8_t*)Serial_RxBuffer2, strlen(Serial_RxBuffer2), HAL_MAX_DELAY);
		ParseAndExecuteCommand((char*)Serial_RxBuffer4);
		RxFlag4 = 0;
	}

	if (PIDflag)
	{
		CCRX_PID_Update();
		CCRY_PID_Update();
		currentCCRX = __HAL_TIM_GET_COMPARE(&htim4,TIM_CHANNEL_4);
		currentCCRY = __HAL_TIM_GET_COMPARE(&htim4,TIM_CHANNEL_3);
		currentCCRX += CCRX_PID.DeltaCCR;
		currentCCRY -= CCRY_PID.DeltaCCR;
//		currentCCRX = (currentCCRX < maxCCRX)? currentCCRX : maxCCRX;
//		currentCCRX = (currentCCRX > minCCRX)? currentCCRX : minCCRX;
//		currentCCRY = (currentCCRY < maxCCRY)? currentCCRY : maxCCRY;
//		currentCCRY = (currentCCRY > minCCRY)? currentCCRY : minCCRY;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, currentCCRX);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, currentCCRY);
		PIDflag = 0;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
