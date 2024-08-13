/*
 * uartcom.cpp
 *
 *  Created on: Aug 12, 2024
 *      Author: 42077
 */

#include "uartcom.hpp"

UartCom* UartCom::instance = nullptr;

UartCom* UartCom::GetInstance(void *uartHW)
{
    if(instance == nullptr)
    {
        static UartCom singletonUartCom;
        instance = &singletonUartCom;
    }
    return instance;
}

void UartCom::StartBuffer()
{

}

void UartCom::StopBuffer()
{

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

void UartCom::UartComCriticalEnter()
{
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void UartCom::UartComCriticalExit()
{
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}






