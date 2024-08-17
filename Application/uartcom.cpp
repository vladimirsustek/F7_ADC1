/*
 * uartcom.cpp
 *
 *  Created on: Aug 12, 2024
 *      Author: 42077
 */

#include <cassert>
#include "uartcom.hpp"

extern "C"
{
#include "usart.h"
}

UartCom* UartCom::instance = nullptr;
UART_HandleTypeDef* UartCom::peripheral = nullptr;

UartCom* UartCom::GetInstance(UARTPeripheral uartHW)
{
	/* When instance was not yet initialized */

	switch(uartHW)
	{
	case UARTPeripheral::F7_UART3 :
	{
	    if(instance == nullptr)
	    {
	        static UartCom singletonUartCom;
	        instance = &singletonUartCom;
	    }
	    if(peripheral == nullptr)
	    {
			peripheral = &huart3;
	    }
	}
	break;
	default : {
		return nullptr;
	}
	}

    return instance;
}

void UartCom::StartBuffer()
{
	assert(HAL_OK == HAL_UART_Receive_IT(peripheral,
			const_cast<uint8_t*>(rx_buff), sizeof(uint8_t)));
}

void UartCom::StopBuffer()
{
	assert(HAL_OK == HAL_UART_AbortReceive(peripheral));
}

uint32_t UartCom::ReadLine(uint8_t* buff, uint32_t buff_size)
{
    if (nullptr == buff || buff_size > UART_RX_BUFF_SIZE || rx_buff_records == 0)
    {
        return 0;
    }
    else
    {
        uint8_t bytes = 0;

        UartComCriticalEnter();

        while(rx_buff_wr_idx != rx_buff_rd_idx)
        {
        	buff[bytes++] = rx_buff[rx_buff_rd_idx];

            if(rx_buff[rx_buff_rd_idx] == '\n')
            {
                rx_buff_rd_idx = (rx_buff_rd_idx + 1) & (UART_RX_BUFF_SIZE - 1);
                break;
            }

            rx_buff_rd_idx = (rx_buff_rd_idx + 1) & (UART_RX_BUFF_SIZE - 1);

            /* When overflow happened, the write pointer was not moved (intentional),
               because this while loop condition would be met, so write pointer is now
               moved after as the first character was processed */
            if(rx_buff_overflow)
            {
                rx_buff_overflow = 0;
                rx_buff_wr_idx = (rx_buff_wr_idx + 1) & (UART_RX_BUFF_SIZE - 1);
            }
        }

        rx_buff_records--;

        UartComCriticalExit();

        return bytes;
    }
}

void UartCom::Write(uint8_t c)
{
	assert(HAL_OK == HAL_UART_Transmit(peripheral,
			&c, sizeof(uint8_t), HAL_MAX_DELAY));
}

void UartCom::Write(uint8_t *buff, uint32_t length)
{
	assert(HAL_OK == HAL_UART_Transmit(peripheral,
			buff, length, HAL_MAX_DELAY));
}

void UartCom::UartComCriticalEnter()
{
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}

void UartCom::UartComCriticalExit()
{
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}
