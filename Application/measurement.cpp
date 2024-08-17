/*
 * measurement.cpp
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */
#include "measurement.hpp"

#include <cassert>

extern "C"
{
#include "tim.h"
#include "adc.h"
}

Measurement* Measurement::instance = nullptr;

Measurement* Measurement::GetInstance()
{
	if(instance == nullptr)
	{
		static Measurement singletonMeasurement;
		instance = &singletonMeasurement;
	}
    return instance;
}

void Measurement::StartMeasurement()
{
  /* TIM-Triggered DMA ADC Continuous conversion
   * For multiple ADC channels on a more complex
   * MCU (STM32F767) is needed to initialize more
   * "ADC's RANKS" (HAL_ADC_ConfiChannel..)
   * 0) ADC must support concrete timer to be trigger source (TRGO)
   * 1) Timer must have selected "Output Update Event .. TRGO"
   * 2) ADC must have enabled triggers source (TIMER X output event)
   * 2) DMA must be initialized before the ADC
   * 3) Timer must be started before the ADC DMA*/

	assert(HAL_OK == HAL_TIM_Base_Start(&htim1));
	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 2));

	/* Temperature sensor characteristics, datasheet - production data,
	 * STM32F765xx STM32F767xx STM32F768Ax STM32F769xx, 6.3.25 (Page 168) */
	/* TS ADC raw data acquired at temperature of 30 °C, VDDA= 3.3 V */
	TS_CAL1 = (float)*((uint16_t*)0x1FF0F44C); /* 0x1FF0F44C - 0x1FF0F44D*/
	/* TS ADC raw data acquired at temperature of 110 °C, VDDA= 3.3 V */

	TS_CAL2 = (float)*((uint16_t*)0x1FF0F44E); /* 0x1FF0F44E - 0x1FF0F44F*/

	V_CAL21 = TS_CAL2 - TS_CAL1;
	T_CAL21 = T_CAL2 - T_CAL1;
	T_CAL_R = T_CAL21/V_CAL21;
}

void Measurement::StopMeasurement()
{
	assert(HAL_OK == HAL_TIM_Base_Stop(&htim1));
	assert(HAL_OK == HAL_ADC_Stop(&hadc1));
}

void Measurement::FifoInsert(uint32_t element)
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

uint32_t Measurement::FifoGetAverage(void)
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

float Measurement::GetTemperature()
{
	  float rawadc = static_cast<float>(FifoGetAverage());
	  float v_sense = (float)(rawadc*VBAT/static_cast<float>(ADC_MAX));
	  float temp1 = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET;
	  return temp1;
}

float Measurement::GetCalibTemperature()
{
	  float rawadc = static_cast<float>(FifoGetAverage());
	  float temp2 = T_CAL_R*rawadc + T_CAL1 - T_CAL_R*TS_CAL1;
	  return temp2;
}
