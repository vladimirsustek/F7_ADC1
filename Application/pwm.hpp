/*
 * pwm.hpp
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */

#ifndef PWM_HPP_
#define PWM_HPP_

extern "C"
{
#include "tim.h"
}

class Pwm
{
public:
	static const uint32_t PWM_MAX_PERIOD = 999;
	static Pwm* GetInstance();
	void PwmStartCh1();
	void PwmStartCh2();
	void PwmStopCh1();
	void PwmStopCh2();
	void PwmSetPwmCh1(uint32_t dc);
	void PwmSetPwmCh2(uint32_t dc);
private:
	Pwm() = default;
	~Pwm() = default;
	static Pwm* instance;
};


#endif /* PWM_HPP_ */
