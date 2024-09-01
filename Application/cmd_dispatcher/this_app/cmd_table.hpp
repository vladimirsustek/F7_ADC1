/*
 * cmd_table.hpp
 *
 *  Created on: Sep 1, 2024
 *      Author: 42077
 */

#ifndef CMD_DISPATCHER_THIS_APP_CMD_TABLE_HPP_
#define CMD_DISPATCHER_THIS_APP_CMD_TABLE_HPP_

#include "cmd_pwm.hpp"
#include "cmd_gpio.hpp"
#include "cmd_meas.hpp"

constexpr uint32_t HELP_MAX_LINE_LNG = 64u;
constexpr uint32_t CMD_TABLE_SIZE = 8u;

const CmdDisp_t cmdTable[CMD_TABLE_SIZE] = {
/*01*/		{method_set,		cmd_blue_led,			cmdWriteBlueLED},
/*02*/		{method_set,		cmd_red_led,			cmdWriteRedLED},
/*03*/		{method_read,		cmd_temperature1,		cmdReadTemperature1},
/*04*/		{method_read,		cmd_temperature2,		cmdReadTemperature2},
/*05*/		{method_enable,		cmd_pwm1,				cmdEnablePwmCh1},
/*06*/		{method_enable,		cmd_pwm2,				cmdEnablePwmCh2},
/*07*/		{method_set,		cmd_pwm1,				cmdSetPwmCh1},
/*08*/		{method_set,		cmd_pwm2,				cmdSetPwmCh2}
};

#endif /* CMD_DISPATCHER_THIS_APP_CMD_TABLE_HPP_ */
