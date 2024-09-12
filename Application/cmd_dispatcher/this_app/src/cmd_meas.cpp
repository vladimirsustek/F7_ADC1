/*
 * cmd_meas.cpp
 *
 *  Created on: Aug 24, 2024
 *      Author: 42077
 */

#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "measurement.hpp"
#include "uartcom.hpp"
#include "cmd_utils.hpp"

uint32_t ReadTemperature1(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG + CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	Measurement* measurement = Measurement::GetInstance();

	char buffer[8];

	float ftemp = measurement->GetTemperature();

	ftemp *= 100;

	int32_t itemp = static_cast<int32_t>(ftemp);

	uint32_t length = i32toStr(itemp, buffer, 8);
	buffer[length] = CMD_EOL;
	length += CMD_EOL_LNG;

	assert(length <= 8);

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    uart->Write(reinterpret_cast<uint8_t*>(buffer), length);

	return CMD_RET_OK;
}

uint32_t ReadTemperature2(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG + CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	Measurement* measurement = Measurement::GetInstance();

	char buffer[8] = {0};

	float ftemp = measurement->GetCalibTemperature();

	ftemp *= 100;

	int32_t itemp = static_cast<int32_t>(ftemp);

	uint32_t length = i32toStr(itemp, buffer, 8);
	buffer[length] = CMD_EOL;
	length += CMD_EOL_LNG;

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    uart->Write(reinterpret_cast<uint8_t*>(buffer), length);

	return CMD_RET_OK;
}
