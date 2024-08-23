/*
 * pwm.cpp
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */

extern "C"
{
#include "tim.h"
}

#include "pwm.hpp"

Pwm* instance = nullptr;

Pwm* Pwm::GetInstance()
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
	if(instance == nullptr)
	{
		static Pwm singletonPwm;
		instance = &singletonPwm;

	}
	htim2.Instance->ARR = 999u;
    return instance;
}

void Pwm::PwmStartCh1()
{
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void Pwm::PwmStartCh2()
{
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Pwm::PwmStopCh1()
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void Pwm::PwmStopCh2()
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

void Pwm::PwmSetPwmCh1(uint32_t dc)
{
	htim2.Instance->CCR1 = dc;
}

void Pwm::PwmSetPwmCh2(uint32_t dc)
{
	htim2.Instance->CCR2 = dc;
}

