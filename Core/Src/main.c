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
const uint32_t VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;
const uint32_t ADC_CH_RANK_1 = 0;
const uint32_t ADC_CH_RANK_2 = 1;

volatile uint8_t rx_buff[UART_RX_BUFF_SIZE];
volatile uint8_t rx_buff_wr_idx;
volatile uint8_t rx_buff_rd_idx;
volatile uint8_t rx_buff_records;
volatile uint8_t rx_buff_overflow;

volatile uint32_t adc[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t _UART_ReadLine(uint8_t* pData, uint16_t size);
static void uart_enter_critical(void);
static void uart_exit_critical(void);
uint16_t SetPWM(uint8_t* pStrCmd, const uint8_t lng);
uint16_t GetTemperature(uint8_t* pStrCmd, const uint8_t lng);
uint16_t ControlGreenLED(uint8_t* pStrCmd, const uint8_t lng);
uint32_t lifo_getAverage(void);
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
	uint32_t prevTick = 0;
	uint32_t btnDebouncer = 0;
	uint32_t tempReadoutAutomatic = 0;
	uint32_t tempReadoutAutomaticTick = 0;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buff, sizeof(uint8_t));

  htim2.Instance->ARR = 999u;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t buffer[64] = {0};
	  uint16_t size = 0;
	  if((size = _UART_ReadLine(buffer, 1)) > 0)
	  {
		  if (SetPWM(buffer, size) &&
		  GetTemperature(buffer, size) &&
		  ControlGreenLED(buffer, size))
		  {
			  printf("Error\n");
		  }
	  }

	  if(HAL_GetTick() > prevTick + 10)
	  {
		  prevTick = HAL_GetTick();
		  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
		  {
			  btnDebouncer++;
		  }
		  else
		  {
			  btnDebouncer = (btnDebouncer) ? btnDebouncer - 1 : 0u;
		  }

		  if (btnDebouncer > 10)
		  {
			  tempReadoutAutomatic = (tempReadoutAutomatic) ? 0u : 1u;
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (GPIO_PinState)tempReadoutAutomatic);
		  }
	  }

	  if(HAL_GetTick() > tempReadoutAutomaticTick + 333 && tempReadoutAutomatic)
	  {
		  tempReadoutAutomaticTick = HAL_GetTick();
		  float v_sense = (float)(lifo_getAverage()*VDDA_NOM/ADC_MAX);
		  float temp = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET;
		  printf("T = %3.2f*C\n", temp);
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

#define LIFO_SIZE 8

uint32_t lifo[LIFO_SIZE] = {0};
uint32_t lifo_first = 0;
uint32_t lifo_usage = 0;

void lifo_insertElement(uint32_t element)
{
    if(lifo_usage < (LIFO_SIZE - 1))
    {
        lifo[lifo_usage++] = element;
    }
    else
    {
        lifo[lifo_first] = element;
    }

    lifo_first = (lifo_first + 1) % 8;
}

uint32_t lifo_getAverage(void)
{
    uint8_t cnt = 0;
    uint64_t acc = 0;
    uint8_t idx = (lifo_first - 1) % 8;

    while(cnt < 8)
    {
        acc += lifo[idx];
        idx = (idx + 1) % 8;
        cnt++;
    }

    acc = acc >> 3;

    return acc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		//printf("CH12 = %ldmV\r\n", adc[ADC_CH_RANK_1]*VDDA_NOM/ADC_MAX);

		/* Temperature sensor characteristics, RM0410 Reference manual,
		 * STM32F76xxx and STM32F77xxx ... , 15.10 (Page 464) */
		/*Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25*/
		/* TODO: use calibration values */

		//float v_sense = (float)(adc[ADC_CH_RANK_2]*VDDA_NOM/ADC_MAX);
		//temp[idx++] = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET;

		lifo_insertElement(adc[ADC_CH_RANK_2]);

		//printf("T = %3.2f*C\r\n", temp);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    volatile uint8_t c = huart->Instance->RDR;

    volatile uint8_t next_wr_idx = (rx_buff_wr_idx+1) & (UART_RX_BUFF_SIZE-1);

    if(next_wr_idx != rx_buff_rd_idx && rx_buff_overflow == 0)
    {
        if(c == '\n')
        {
            rx_buff_records++;
        }

        rx_buff[rx_buff_wr_idx] = c;

        rx_buff_wr_idx = next_wr_idx;
    }
    else
    {
        rx_buff[rx_buff_wr_idx] = '\n';
        rx_buff_records++;
        rx_buff_overflow = 1;
    }

    HAL_UART_Receive_IT(&huart3, (uint8_t*)(rx_buff + rx_buff_wr_idx), sizeof(uint8_t));
}

uint16_t _UART_ReadLine(uint8_t* pData, uint16_t size)
{
    if (NULL == pData || size > UART_RX_BUFF_SIZE || rx_buff_records == 0)
    {
        return 0;
    }
    else
    {
        uint8_t bytes = 0;

        uart_enter_critical();

        while(rx_buff_wr_idx != rx_buff_rd_idx)
        {
            pData[bytes++] = rx_buff[rx_buff_rd_idx];

            if(rx_buff[rx_buff_rd_idx] == '\n')
            {
                rx_buff_rd_idx = (rx_buff_rd_idx + 1) & (UART_RX_BUFF_SIZE - 1);
                break;
            }

            rx_buff_rd_idx = (rx_buff_rd_idx + 1) & (UART_RX_BUFF_SIZE - 1);

            /* When overflow happened, the write pointer was not moved (intentional),
               because this while loop condition would be met, so write pointer is now
               moved after as the first character was processed */
            if(rx_buff_overflow)
            {
                rx_buff_overflow = 0;
                rx_buff_wr_idx = (rx_buff_wr_idx + 1) & (UART_RX_BUFF_SIZE - 1);
            }
        }

        rx_buff_records--;

        uart_exit_critical();

        return bytes;
    }
}

static void uart_enter_critical(void) {
    HAL_NVIC_DisableIRQ(USART3_IRQn);

}

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

uint16_t GetTemperature(uint8_t* pStrCmd, const uint8_t lng)
{
	if(memcmp(pStrCmd, "CHIPT", 5u) == 0)
	{

		float v_sense = (float)(lifo_getAverage()*VDDA_NOM/ADC_MAX);
		float temp = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET;

		printf("T = %3.2f*C\n", temp);
		return 0;
	}
	else
	{
		return (uint16_t)(-1);
	}
}

uint16_t ControlGreenLED(uint8_t* pStrCmd, const uint8_t lng)
{
	if(memcmp(pStrCmd, "LEDG_", 5u) == 0)
	{
		if(pStrCmd[5] == '0')
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
			printf((char*)pStrCmd);
			return 0;
		}
		if(pStrCmd[5] == '1')
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			printf((char*)pStrCmd);
			return 0;
		}
	}

	return (uint16_t)(-1);

}
/* @brief Enable UART interrupt (so its ISR)
 *
 *
 */
static void uart_exit_critical(void) {
    HAL_NVIC_EnableIRQ(USART3_IRQn);
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
