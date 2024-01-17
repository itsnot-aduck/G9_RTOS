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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dht11.h"
#include "delay_timer.h"
#include "lcd_i2c.h"
#include "hrf05.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Display mode */
typedef enum
{
	DISPLAY_DISTANCE,
	DISPLAY_TEMP,
	DISPLAY_HUMI,
	DISPLAY_ALL
} DisplayMode_t;

typedef struct
{
	float humi;
	float dist;
	float temp;
} SenseData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RX_LEN 				13
#define DISPLAY_COMMAND_LEN 	11
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadDHT */
osThreadId_t ReadDHTHandle;
const osThreadAttr_t ReadDHT_attributes = {
		.name = "ReadDHT",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ReadDist */
osThreadId_t ReadDistHandle;
const osThreadAttr_t ReadDist_attributes = {
		.name = "ReadDist",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for HandleInterrupt */
osThreadId_t HandleInterruptHandle;
const osThreadAttr_t HandleInterrupt_attributes = {
		.name = "HandleInterrupt",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
		.name = "Display",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DataSem */
osSemaphoreId_t DataSemHandle;
const osSemaphoreAttr_t DataSem_attributes = {
		.name = "DataSem"
};
/* Definitions for IRQSem */
osSemaphoreId_t IRQSemHandle;
const osSemaphoreAttr_t IRQSem_attributes = {
		.name = "IRQSem"
};
/* USER CODE BEGIN PV */
/* Sensors Instance */
DHT11_Sensor dht;
LCD_I2C_Name lcd;
SRF05_Device_Name hrf05;

/* Buffers and flags */
uint8_t count;
DHT11_Status dhtStatus;
char temp[18];
uint8_t rxData[20]; /* UART data receiver buffer */
uint8_t rxDataIndex = 0;
DisplayMode_t DisplayMode = DISPLAY_ALL;
/* Task interval */
uint32_t dhtInterval = 1500;
uint32_t distInterval = 1500;

/* Struct data of sensing measurements */
SenseData_t SenseData;
osStatus_t status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void vTask_ReadDHT(void *argument);
void vTask_ReadDist(void *argument);
void vTask_InterruptHandler(void *argument);
void vTask_Display(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

/* Interrupt handler */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		if (rxData[rxDataIndex] == '\n')
		{
			rxData[rxDataIndex] = '\0';
			printf("\nCommand: %s\r\n", rxData);
			rxDataIndex = 0;
			if (osThreadGetPriority(HandleInterruptHandle) != osPriorityHigh)
			{
				osThreadSetPriority(HandleInterruptHandle, osPriorityHigh);
			}
			osSemaphoreRelease(IRQSemHandle);
		}
		else
		{
			rxDataIndex ++;
		}
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataIndex], 1);
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
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	printf("Start\r\n\n");
	DHT11_Init(&dht, DHT_GPIO_Port, DHT_Pin, &htim4);
	LCD_Init(&lcd, &hi2c2, LDC_DEFAULT_ADDRESS, 20, 4);
	SRF05_Init(&hrf05, ECHO_GPIO_Port, ECHO_Pin, TRIG_GPIO_Port, TRIG_Pin);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataIndex], 1);
	LCD_SetCursor(&lcd, 0, 0);
	LCD_WriteString(&lcd, "Hello");
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of DataSem */
	DataSemHandle = osSemaphoreNew(1, 0, &DataSem_attributes);

	/* creation of IRQSem */
	IRQSemHandle = osSemaphoreNew(1, 0, &IRQSem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of ReadDHT */
	ReadDHTHandle = osThreadNew(vTask_ReadDHT, NULL, &ReadDHT_attributes);

	/* creation of ReadDist */
	ReadDistHandle = osThreadNew(vTask_ReadDist, NULL, &ReadDist_attributes);

	/* creation of HandleInterrupt */
	HandleInterruptHandle = osThreadNew(vTask_InterruptHandler, NULL, &HandleInterrupt_attributes);

	/* creation of Display */
	DisplayHandle = osThreadNew(vTask_Display, NULL, &Display_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	htim4.Init.Prescaler = 83;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
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
	huart2.Init.BaudRate = 115200;
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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : TRIG_Pin */
	GPIO_InitStruct.Pin = TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ECHO_Pin */
	GPIO_InitStruct.Pin = ECHO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DHT_Pin */
	GPIO_InitStruct.Pin = DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTask_ReadDHT */
/**
 * @brief Function implementing the ReadDHT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTask_ReadDHT */
void vTask_ReadDHT(void *argument)
{
	/* USER CODE BEGIN vTask_ReadDHT */
	int32_t tick = osKernelGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		if (osSemaphoreGetCount(DataSemHandle) != 0)
		{
			osSemaphoreAcquire(DataSemHandle, osWaitForever);
		}
		tick = tick + dhtInterval;
		printf("vTask_ReadDHT IN: %ld\r\n", osKernelGetTickCount());
		SRF05_Read(&hrf05);
		SenseData.dist = hrf05.Distance;
		printf("vTask_ReadDHT OUT: %ld\r\n\n", osKernelGetTickCount());
		dhtStatus = DHT11_GetData(&dht);
		switch(dhtStatus)
		{
		case DHT11_ERR_CHECKSUM:
			printf("DHT11 ERROR CHECKSUM\r\n");
			break;
		case DHT11_ERR_RESPONSE:
			printf("DHT11 ERROR RESPONSE\r\n");
			break;
		default:
			SenseData.humi = dht.Humi;
			SenseData.temp = dht.Temp;
			break;
		}

		osSemaphoreRelease(DataSemHandle);

		osDelayUntil(tick);
	}
	/* USER CODE END vTask_ReadDHT */
}

/* USER CODE BEGIN Header_vTask_ReadDist */
/**
 * @brief Function implementing the ReadDist thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTask_ReadDist */
void vTask_ReadDist(void *argument)
{
	/* USER CODE BEGIN vTask_ReadDist */
	int32_t tick = osKernelGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		if (osSemaphoreGetCount(DataSemHandle) != 0)
		{
			osSemaphoreAcquire(DataSemHandle, osWaitForever);
		}
		tick = tick + distInterval;
		printf("vTask_ReadDist IN: %ld\r\n", osKernelGetTickCount());
		SRF05_Read(&hrf05);
		SenseData.dist = hrf05.Distance;
		printf("vTask_ReadDist OUT: %ld\r\n\n", osKernelGetTickCount());

		osSemaphoreRelease(DataSemHandle);

		osDelayUntil(tick);
	}
	/* USER CODE END vTask_ReadDist */
}

/* USER CODE BEGIN Header_vTask_InterruptHandler */
/**
 * @brief Function implementing the InterruptHandle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTask_InterruptHandler */
void vTask_InterruptHandler(void *argument)
{
	/* USER CODE BEGIN vTask_InterruptHandler */
	/* Infinite loop */
	for(;;)
	{
		/* Temporary pause others tasks */
		osSemaphoreAcquire(IRQSemHandle, osWaitForever);
		osThreadSuspend(ReadDistHandle);
		osThreadSuspend(ReadDHTHandle);
		osThreadSuspend(DisplayHandle);

		/* Interrupt handler */
		char *command = strtok((char*)rxData, " ");
		char *time = strtok(NULL, " ");
		if (NULL == time){
			if (strcmp((const char*)rxData, "displaytemp") == 0)
			{
				DisplayMode = DISPLAY_TEMP;
				printf("Change Display Mode to DISPLAY_TEMP\r\n\n");
			}
			else if (strcmp((const char*)rxData, "displayhumi") == 0)
			{
				DisplayMode = DISPLAY_HUMI;
				printf("Change Display Mode to DISPLAY_HUMI\r\n\n");
			}
			else if (strcmp((const char*)rxData, "displaydist") == 0)
			{
				DisplayMode = DISPLAY_DISTANCE;
				printf("Change Display Mode to DISPLAY_DIST\r\n\n");
			}
			else if (strcmp((const char*)rxData, "displayboth") == 0)
			{
				DisplayMode = DISPLAY_ALL;
				printf("Change Display Mode to DISPLAY_ALL\r\n\n");
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
		}
		else {
			uint32_t pTime = atoi(time);
			if (strcmp((const char*)command, "timedht1") == 0)
			{
				dhtInterval = pTime;
				printf("Change period of dht time to %ld\r\n", pTime);
			}
			else if (strcmp((const char*)command, "timedist") == 0)
			{
				distInterval = pTime;
				printf("Change period of dist time to %ld\r\n", pTime);
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
		}
		/* let continue others */
		osThreadResume(ReadDistHandle);
		osThreadResume(ReadDHTHandle);
		osThreadResume(DisplayHandle);
	}
	/* USER CODE END vTask_InterruptHandler */
}

/* USER CODE BEGIN Header_vTask_Display */
/**
 * @brief Function implementing the Display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTask_Display */
void vTask_Display(void *argument)
{
	/* USER CODE BEGIN vTask_Display */
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(DataSemHandle, osWaitForever);
		printf("vTask_Display IN: %ld\r\n", osKernelGetTickCount());

		if (dhtStatus == DHT11_OK)
		{
			LCD_Clear(&lcd);
			switch (DisplayMode){
			case DISPLAY_DISTANCE:
				sprintf(temp, "D: %.2f", SenseData.dist);
				LCD_SetCursor(&lcd, 0, 0);
				LCD_WriteString(&lcd, temp);
				printf("Distance: %.2f\r\n", SenseData.dist);
				break;
			case DISPLAY_HUMI:
				sprintf(temp, "H: %.2f", SenseData.humi);
				LCD_SetCursor(&lcd, 0, 0);
				LCD_WriteString(&lcd, temp);
				printf("Humidity: %.2f\r\n", SenseData.humi);
				break;
			case DISPLAY_TEMP:
				sprintf(temp, "T: %.2f", SenseData.temp);
				LCD_SetCursor(&lcd, 0, 0);
				LCD_WriteString(&lcd, temp);
				printf("Temperature: %.2f\r\n", SenseData.temp);
				break;
			case DISPLAY_ALL:
				sprintf(temp, "T: %.2f D: %.2f", SenseData.temp, SenseData.dist);
				LCD_SetCursor(&lcd, 0, 0);
				LCD_WriteString(&lcd, temp);
				sprintf(temp, "H: %.2f", SenseData.humi);
				LCD_SetCursor(&lcd, 0, 1);
				LCD_WriteString(&lcd, temp);
				printf("Distance: %.2f\r\n", SenseData.dist);
				printf("Humidity: %.2f\r\n", SenseData.humi);
				printf("Temperature: %.2f\r\n", SenseData.temp);
				break;
			}
		}
		printf("vTask_Display OUT: %ld\r\n\n", osKernelGetTickCount());
	}
	/* USER CODE END vTask_Display */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
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
