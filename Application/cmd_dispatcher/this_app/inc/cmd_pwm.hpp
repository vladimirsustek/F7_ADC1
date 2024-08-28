/*
 * cmd_pwm.hpp
 *
 *  Created on: Aug 23, 2024
 *      Author: 42077
 */

#ifndef CMD_PWM_HPP_
#define CMD_PWM_HPP_

uint32_t EnablePwmCh1(const uint8_t* const pStrCmd, const uint8_t lng);
uint32_t EnablePwmCh2(const uint8_t* const pStrCmd, const uint8_t lng);
uint32_t SetPwmCh1(const uint8_t* const pStrCmd, const uint8_t lng);
uint32_t SetPwmCh2(const uint8_t* const pStrCmd, const uint8_t lng);

const CommandArg DEC_UI16_PWM =
{
		DEC_UI16, Pwm::PWM_MIN_PERIOD, Pwm::PWM_MAX_PERIOD
};

const DispatcherCommand_t cmdEnablePwmCh1 =
{
		EnablePwmCh1,
		DEC_UI16_BOOL
};

const DispatcherCommand_t cmdEnablePwmCh2 =
{
		EnablePwmCh2,
		DEC_UI16_BOOL
};

const DispatcherCommand_t cmdSetPwmCh1 =
{
		SetPwmCh1,
		DEC_UI16_PWM
};

const DispatcherCommand_t cmdSetPwmCh2 =
{
		SetPwmCh2,
		DEC_UI16_PWM
};

#endif /* CMD_PWM_HPP_ */
