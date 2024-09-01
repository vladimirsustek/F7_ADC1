/*
 * cmd_ioport.hpp
 *
 *  Created on: Sep 1, 2024
 *      Author: 42077
 */

#ifndef CMD_DISPATCHER_THIS_APP_CMD_IOPORT_HPP_
#define CMD_DISPATCHER_THIS_APP_CMD_IOPORT_HPP_

#include <cstdint>
#include <uartcom.hpp>

static UartCom *uart = nullptr;

void cmdDispOutInit()
{
	uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);
}

void cmdDispOutMsg(char *pMsg, uint32_t lng)
{
	uart->Write(reinterpret_cast<uint8_t*>(pMsg), lng);
}

void cmdDispOutMsg(const char *pMsg, uint32_t lng)
{
	uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(pMsg)), lng);
}

#endif /* CMD_DISPATCHER_THIS_APP_CMD_IOPORT_HPP_ */
