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
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */
#if CALIBRATION_DATA_INTERPOLATION
float TS_CAL1 = 0;
float TS_CAL2 = 0;
float V_REFIN_CAL = 0;
#endif
const float V25 = 760; // V at 25°C
const float AVG_SLOPE = 2.5; //mV/°C
const float T_OFFSET = 25.0;
const uint32_t VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;
const uint32_t ADC_CH_RANK_1 = 0;
const uint32_t ADC_CH_RANK_2 = 1;
uint32_t adc[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
#if CALIBRATION_DATA_INTERPOLATION
	/* Temperature sensor characteristics, datasheet - production data,
	 * STM32F765xx STM32F767xx STM32F768Ax STM32F769xx, 6.3.25 (Page 168) */
	/* TS ADC raw data acquired at temperature of 30 °C, VDDA= 3.3 V */
	TS_CAL1 = (float)*((uint16_t*)0x1FF0F44C); /* 0x1FF0F44C - 0x1FF0F44D*/
	/* TS ADC raw data acquired at temperature of 110 °C, VDDA= 3.3 V */

	TS_CAL2 = (float)*((uint16_t*)0x1FF0F44E); /* 0x1FF0F44E - 0x1FF0F44F*/
	/* Reference voltage, datasheet - production data,
	 * STM32F765xx STM32F767xx STM32F768Ax STM32F769xx, 6.3.27 (Page 169) */
	V_REFIN_CAL = (float)*((uint16_t*)0x1FF0F44A); /* 0x1FF0F44A - 0x1FF0F44B*/
#endif
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
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* TIM-Triggered DMA ADC Continuous conversion
   * For multiple ADC channels on a more complex
   * MCU (STM32F767) is needed to initialize more
   * "ADC's RANKS" (HAL_ADC_ConfiChannel..)
   * 0) ADC must support concrete timer to be trigger source (TRGO)
   * 1) Timer must have selected "Output Update Event .. TRGO"
   * 2) ADC must have enabled triggers source (TIMER X output event)
   * 2) DMA must be initialized before the ADC
   * 3) Timer must be started before the ADC DMA*/
  HAL_TIM_Base_Start(&htim1);
  HAL_ADC_Start_DMA(&hadc1, adc, 2);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

/*
 * Calculate the temperature using the following formula:
Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25
Where:
– V25 = VSENSE value for 25° C
– Avg_Slope = average slope of the temperature vs. VSENSE curve (given in mV/°C
or µV/°C)
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
		printf("CH12 = %ldmV\r\n", adc[ADC_CH_RANK_1]*VDDA_NOM/ADC_MAX);

		/* Temperature sensor characteristics, RM0410 Reference manual,
		 * STM32F76xxx and STM32F77xxx ... , 15.10 (Page 464) */
		/*Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25*/
		/* TODO: use calibration values */

		float v_sense = (float)(adc[1]*VDDA_NOM/ADC_MAX);
		float temp = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET;
		printf("T = %3.2f*C\r\n", temp);
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
