/*
 * cmd_meas.hpp
 *
 *  Created on: Aug 24, 2024
 *      Author: 42077
 */

#ifndef CMD_MEAS_HPP_
#define CMD_MEAS_HPP_

uint32_t ReadTemperature1(const uint8_t* const pStrCmd, const uint8_t lng);
uint32_t ReadTemperature2(const uint8_t* const pStrCmd, const uint8_t lng);

const DispatcherCommand_t cmdReadTemperature1
{
	ReadTemperature1,
	NO_ARG
};

const DispatcherCommand_t cmdReadTemperature2
{
	ReadTemperature2,
	NO_ARG
};

#endif /* CMD_MEAS_HPP_ */
