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

constexpr uint32_t UART_RX_BUFF_SIZE = 64u;
constexpr uint32_t SUPPORTED_PERIPHERALS = 1u;

enum UARTPeripheral
{
	F7_UART3 = 0,
};

class UartCom
{
    friend void UartCallback(UartCom & uartcom, UART_HandleTypeDef *huart);
public:
	static UartCom* GetInstance(UARTPeripheral uartHW);
	void StartBuffer();
	void StopBuffer();
	void Write(uint8_t c);
	void Write(uint8_t *buff, uint32_t length);
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
    static UART_HandleTypeDef* peripheral;

};

#endif /* UARTCOM_HPP_ */
