
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "STM_MY_LCD16X2.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t taskCount[3];
volatile uint32_t ulIdleCycleCount = 0UL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void vContinuousProcessingTask(void* pvParameters);
void vPeriodicTask(void* pvParameters);
void lcdLoadingIndicator(void);
void vApplicationIdleHook( void );
void vTaskFunction(void *pvParameters);
void vTask1(void* pvParameters);
void vTask2(void* pvParameters);
static void vReceiverTaskTest( void *pvParameters );
static void vReceiverTask( void *pvParameters );

void vSenderTask1( void *pvParameters );
void vSenderTask2( void *pvParameters );

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t LED[] = {0, LED1_Pin, LED2_Pin, LED3_Pin};
TaskHandle_t xTask2Handle = NULL;
QueueHandle_t xQueue;
static QueueHandle_t xQueue1 = NULL, xQueue2 = NULL;

static QueueSetHandle_t xQueueSet = NULL;




typedef enum
{
	eSender1,
	eSender2,
} DataSource_t;

typedef struct
{
	uint8_t ucValue;
	DataSource_t eDataSource;
} Data_t;

// static Data_t xStructsToSend[ 2 ] =
// {
// 	{ 100, eSender1 },
// 	{ 200, eSender2 }
// };
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	taskCount[1] = 0;
	taskCount[2] = 0;
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	LCD1602_Begin8BIT(RS_GPIO_Port, RS_Pin, EN_Pin, D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
//  LCD1602_noBlink();
//  LCD1602_noCursor();

	LCD1602_clear();

	xQueue = xQueueCreate( 3, sizeof( Data_t ) );
	xQueue1 = xQueueCreate(1, sizeof(char*));
	xQueue2 = xQueueCreate(1, sizeof(char*));

	xQueueAddToSet(xQueue1, xQueueSet);
	xQueueAddToSet(xQueue2, xQueueSet);

	xTaskCreate(vSenderTask1, "Sender1", 1000, NULL, 1, NULL);
	xTaskCreate(vSenderTask2, "Sender2", 1000, NULL, 1, NULL);

	xTaskCreate(vReceiverTask, "Receiver", 1000, NULL, 2, NULL);

	vTaskStartScheduler();

	for (;;);


}

void vSenderTask1( void *pvParameters )
{
	const TickType_t xBlockTime = pdMS_TO_TICKS(100);
	const char * const pcMessage = "vSenderTask1";



	for (;;)
	{
		vTaskDelay( xBlockTime );

		xQueueSend( xQueue1, &pcMessage, 0 );
	}
}

void vSenderTask2( void *pvParameters )
{
	const TickType_t xBlockTime = pdMS_TO_TICKS(200);
	const char * const pcMessage = "vSenderTask2";



	for (;;)
	{
		
		vTaskDelay( xBlockTime );

		xQueueSend( xQueue2, &pcMessage, 0 );
	}
}

static void vReceiverTaskTest( void *pvParameters )
{
	char *pcReceivedString;
	for (;;)
	{
		xQueueReceive(xQueue2, &pcReceivedString, 4);
		LCD1602_setCursor(2, 1);
		LCD1602_print(pcReceivedString);
	}
}


static void vReceiverTask( void *pvParameters )
{
	QueueHandle_t xQueueThatContainsData;
	char *pcReceivedString;

	for (;;)
	{


		xQueueThatContainsData = (QueueHandle_t)xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

		xQueueReceive(xQueueThatContainsData, &pcReceivedString, 0);

		LCD1602_setCursor(2, 1);
		LCD1602_print(pcReceivedString);
	}
}

void vTask1(void* pvParameters)
{



	for (;;)
	{
		HAL_GPIO_WritePin(GPIOC, LED[1] | LED[2] | LED[3], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, LED[1], GPIO_PIN_SET);
		LCD1602_setCursor(1, 1);
		LCD1602_print("Task 1 running");
		LCD1602_setCursor(2, 1);
		LCD1602_print("              ");
		HAL_Delay(pdMS_TO_TICKS(100));
		xTaskCreate(vTask2, "Task 2", 1000, NULL, 2, &xTask2Handle);
		vTaskDelay(1);
	}
}

void vTask2(void* pvParameters)
{

	for (;;)
	{
		HAL_GPIO_WritePin(GPIOC, LED[1] | LED[2] | LED[3], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, LED[2], GPIO_PIN_SET);
		LCD1602_setCursor(1, 1);
		LCD1602_print("Task 2 running");
		LCD1602_setCursor(2, 1);
		LCD1602_print("Delete Task 2");
		HAL_Delay(pdMS_TO_TICKS(100));
		vTaskDelete(xTask2Handle);
	}
}

void vTaskFunction(void *pvParameters)
{
	int numb = (int)pvParameters;
	char snum[1];
	char str[16] = "";
	sprintf(snum, "%d", numb);
	strcat(str, snum);
	strcat(str, " ");
	const TickType_t xDelay250ms = pdMS_TO_TICKS(250);

	for (;;)
	{
		LCD1602_setCursor(numb, 1);
		LCD1602_print(str);
		LCD1602_PrintInt(ulIdleCycleCount);

		vTaskDelay(xDelay250ms);
	}
}

/* Idle hook functions MUST be called vApplicationIdleHook(), takes no parameters,
and return void. */
void vApplicationIdleHook( void )
{
	if (ulIdleCycleCount % 100000 == 0)
		HAL_GPIO_TogglePin(GPIOC, LED[3]);
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;
}

/*****************************************/

/* Continuous Processing Task */

void vContinuousProcessingTask(void* pvParameters)
{
	int numb = (int) pvParameters;
	char snum[1];
	sprintf(snum, "%d", numb);
	char str[16] = "";
	strcat(str, snum);
	strcat(str, " ");
	HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED[numb], GPIO_PIN_SET);
	for (;;)
	{
		taskCount[numb]++;
		if ((taskCount[numb] % 500000) == 0)
		{
			LCD1602_setCursor(numb, 1);
			LCD1602_print(str);
			LCD1602_PrintInt(taskCount[numb] % (numb * 2018 + 1));
		}
	}
}

/******************************/

/* Periodic Task */
void vPeriodicTask(void* pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xDelay500ms = pdMS_TO_TICKS(5000);

	xLastWakeTime = xTaskGetTickCount();
	HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);

	for (;;)
	{
		LCD1602_clear();
		LCD1602_print("PT running");
		HAL_Delay(pdMS_TO_TICKS(1000));
		LCD1602_clear();
		vTaskDelayUntil(&xLastWakeTime, xDelay500ms);
	}
}

/******************************/

/* Indicating a task is running */

void lcdLoadingIndicator(void)
{

}


/******************************/


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, D0_Pin | D1_Pin | D2_Pin | D3_Pin
	                  | D4_Pin | D5_Pin | D6_Pin | D7_Pin
	                  | EN_Pin | RW_Pin | RS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED3_Pin | LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
	                         D4_Pin D5_Pin D6_Pin D7_Pin
	                         EN_Pin RW_Pin RS_Pin */
	GPIO_InitStruct.Pin = D0_Pin | D1_Pin | D2_Pin | D3_Pin
	                      | D4_Pin | D5_Pin | D6_Pin | D7_Pin
	                      | EN_Pin | RW_Pin | RS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BUTTON3_Pin BUTTON2_Pin BUTTON1_Pin */
	GPIO_InitStruct.Pin = BUTTON3_Pin | BUTTON2_Pin | BUTTON1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void StartDefaultTask(void const * argument)
{

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
