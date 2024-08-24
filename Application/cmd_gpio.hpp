/*
 * cmd_gpio.hpp
 *
 *  Created on: Aug 24, 2024
 *      Author: 42077
 */

#ifndef CMD_GPIO_HPP_
#define CMD_GPIO_HPP_

uint32_t WriteBlueLED(const uint8_t* const pStrCmd, const uint8_t lng);
uint32_t WriteRedLED(const uint8_t* const pStrCmd, const uint8_t lng);

const DispatcherCommand_t cmdWriteBlueLED =
{
		WriteBlueLED,
		DEC_UI16_BOOL
};

const DispatcherCommand_t cmdWriteRedLED =
{
		WriteRedLED,
		DEC_UI16_BOOL
};

#endif /* CMD_GPIO_HPP_ */
