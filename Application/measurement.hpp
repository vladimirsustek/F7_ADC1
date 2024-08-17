/*
 * measurement.hpp
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */

#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <cstdint>

extern "C"
{
#include "adc.h"
}

constexpr uint32_t LIFO_SIZE = 8u;
constexpr float V25 = 760; // V at 25°C
constexpr float AVG_SLOPE = 2.5; //mV/°C
constexpr float T_OFFSET = 25.0;
constexpr float VDDA_NOM = 3300;
constexpr uint32_t ADC_MAX = 4095;
constexpr uint32_t ADC_CH_RANK_1 = 0;
constexpr uint32_t ADC_CH_RANK_2 = 1;
constexpr float VBAT = 3300.0;
constexpr float T_CAL1 = 30.0;
constexpr float T_CAL2 = 110.0;


class Measurement
{
public:
	static Measurement* GetInstance();
	void StartMeasurement();
	void StopMeasurement();
	uint32_t FifoGetAverage();
	float GetTemperature();
	float GetCalibTemperature();
private:
	Measurement() = default;
	~Measurement() = default;
	void FifoInsert(uint32_t element);
	static Measurement* instance;
	volatile uint32_t adc[2] = {0};
	uint32_t lifo[LIFO_SIZE] = {0};
	uint32_t lifo_first = 0;
	uint32_t lifo_usage = 0;
	float TS_CAL1;
	float TS_CAL2;
	float T_CAL21;
	float V_CAL21;
	float T_CAL_R;

	friend void AdcCallback(Measurement & measurement, ADC_HandleTypeDef* hadc);
};

#endif /* MEASUREMENT_HPP_ */
