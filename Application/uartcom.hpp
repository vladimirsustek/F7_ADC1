/*
 * uartcom.hpp
 *
 *  Created on: Aug 12, 2024
 *      Author: 42077
 */

#ifndef UARTCOM_HPP_
#define UARTCOM_HPP_

extern "C"
{
#include "stm32f7xx_hal.h"
}

namespace
{
	constexpr uint32_t UART_RX_BUFF_SIZE = 64u;
}

class UartCom
{
public:
	UartCom* GetInstance(void *uartHW);
	void StartBuffer();
	void StopBuffer();
	uint32_t ReadLine(uint8_t *buff, uint32_t buff_size);

private:
	UartCom() = default;
	~UartCom() = default;
	void UartComCriticalEnter();
	void UartComCriticalExit();
	volatile uint8_t rx_buff[UART_RX_BUFF_SIZE];
	volatile uint8_t rx_buff_wr_idx;
	volatile uint8_t rx_buff_rd_idx;
	volatile uint8_t rx_buff_records;
	volatile uint8_t rx_buff_overflow;
    static UartCom* instance;
};

#endif /* UARTCOM_HPP_ */
