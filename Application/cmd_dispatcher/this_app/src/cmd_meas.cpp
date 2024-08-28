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

uint32_t ReadTemperature1(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG + CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	Measurement* measurement = Measurement::GetInstance();

	char buffer[8] = {0};

	uint32_t length = sprintf(buffer, "%+3.2f\n",  measurement->GetTemperature());

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

	uint32_t length = sprintf(buffer, "%+3.2f\n",  measurement->GetCalibTemperature());

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    uart->Write(reinterpret_cast<uint8_t*>(buffer), length);

	return CMD_RET_OK;
}
