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
#include "adc.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "app_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFF_SIZE       (uint8_t)(64u)
#define PWM_CMD_SIZE			(uint8_t)(9u) //PA03_999\n
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if CALIBRATION_DATA_INTERPOLATION
float TS_CAL1 = 0;
float TS_CAL2 = 0;
float V_REFIN_CAL = 0;
#endif
const float V25 = 760; // V at 25°C
const float AVG_SLOPE = 2.5; //mV/°C
const float T_OFFSET = 25.0;
const float VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;
const uint32_t ADC_CH_RANK_1 = 0;
const uint32_t ADC_CH_RANK_2 = 1;

volatile uint8_t rx_buff[UART_RX_BUFF_SIZE];
volatile uint8_t rx_buff_wr_idx;
volatile uint8_t rx_buff_rd_idx;
volatile uint8_t rx_buff_records;
volatile uint8_t rx_buff_overflow;

volatile uint32_t adc[2] = {0};

float vbat = 3300.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t _UART_ReadLine(uint8_t* pData, uint16_t size);
uint16_t SetPWM(uint8_t* pStrCmd, const uint8_t lng);
uint16_t GetTemperature(uint8_t* pStrCmd, const uint8_t lng);
uint16_t ControlGreenLED(uint8_t* pStrCmd, const uint8_t lng);
uint32_t lifo_getAverage(void);

#define T_CAL1 (float)(30)
#define T_CAL2 (float)(110)

float V_CAL21;
float T_CAL21 = (T_CAL2 - T_CAL1);
float T_CAL_R;
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
	/* Temperature sensor characteristics, datasheet - production data,
	 * STM32F765xx STM32F767xx STM32F768Ax STM32F769xx, 6.3.25 (Page 168) */
	/* TS ADC raw data acquired at temperature of 30 °C, VDDA= 3.3 V */
	float TS_CAL1 = (float)*((uint16_t*)0x1FF0F44C); /* 0x1FF0F44C - 0x1FF0F44D*/
	/* TS ADC raw data acquired at temperature of 110 °C, VDDA= 3.3 V */

	float TS_CAL2 = (float)*((uint16_t*)0x1FF0F44E); /* 0x1FF0F44E - 0x1FF0F44F*/
	/* Reference voltage, datasheet - production data,
	 * STM32F765xx STM32F767xx STM32F768Ax STM32F769xx, 6.3.27 (Page 169) */
	//float V_REFIN_CAL = (float)*((uint16_t*)0x1FF0F44A); /* 0x1FF0F44A - 0x1FF0F44B*/

	V_CAL21 = TS_CAL2 - TS_CAL1;
	T_CAL_R = T_CAL21/V_CAL21;

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  noreturn_app();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t SetPWM(uint8_t* pStrCmd, const uint8_t lng)
{
/*
 *  					CN7
						|--|
						|xx|
						|xx|
						|xx|
						|xx|
	TIM2_CH1 (PB15)		|1x|
						|xx|
						|xx|
	TIM2_CH2 (PB03)		|2x| -
						|xx|
						|xx|
						|--|
*/
	const uint8_t CMD_ARG_OFFSET = 5u; //PA03_

	if(lng != PWM_CMD_SIZE)
	{
        return (uint16_t)(-1);
    }

	for(uint8_t idx = 0; idx < 3; idx++)
	{
		if(pStrCmd[CMD_ARG_OFFSET+idx] > '9' || pStrCmd[CMD_ARG_OFFSET+idx] < '0')
		{
			return (uint16_t)(-1);
		}
	}

    uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 0] - '0')*100;
    period += (pStrCmd[CMD_ARG_OFFSET + 1] - '0')*10;
    period += (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*1;

    if(period > 999)
    {
        return (uint16_t)(-1);
    }

    //TIM2_CH1
    if(memcmp(pStrCmd, "PA15", 4u) == 0)
    {
    	htim2.Instance->CCR1 = period;
    	printf("PA15_%03lu\n", period);

    }

    //TIM2_CH2
    if(memcmp(pStrCmd, "PB03", 4u) == 0)
    {
    	htim2.Instance->CCR2 = period;
    	printf("PB03_%03lu\n", period);
    }


    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_cb(huart);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_cb(hadc);
}

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
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
