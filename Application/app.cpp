/*
 * main.cpp
 *
 *  Created on: Aug 12, 2024
 *      Author: 42077
 */
#include "uartcom.hpp"
#include "measurement.hpp"
#include "pwm.hpp"

#include "app_interface.h"
#include "cmd_dispatcher.hpp"
#include "st_callbacks.hpp"

extern "C"
{
#include "usart.h"
}

uint8_t buffer[UART_RX_BUFF_SIZE];
uint32_t length;


void noreturn_app(void)
{
	UartCom* uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

	Measurement* measurement = Measurement::GetInstance();

    CommandDispatcher dispatcher = CommandDispatcher();

    /* Ensure PWM is really disabled on reset */
    Pwm* pwm = Pwm::GetInstance();

    pwm->PwmSetPwmCh1(Pwm::PWM_MIN_PERIOD);
    pwm->PwmSetPwmCh2(Pwm::PWM_MIN_PERIOD);
    pwm->PwmStopCh1();
    pwm->PwmStopCh2();


	uart->StartBuffer();
	measurement->StartMeasurement();

	while(true)
	{
		if((length = uart->ReadLine(buffer, UART_RX_BUFF_SIZE)))
		{
            buffer[length] = 0u;
            uart->Write(buffer, length);
            dispatcher.Dispatch(buffer, length);
		}
	}
}

void uart_cb(void * handle)
{
	UART_HandleTypeDef *huart = reinterpret_cast<UART_HandleTypeDef*>(handle);

	if(huart->Instance == USART3)
	{
		UartCom* uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);
		UartCallback(*uart, huart);
	}
}

void adc_cb(void * handle)
{
	ADC_HandleTypeDef *hadc = reinterpret_cast<ADC_HandleTypeDef*>(handle);

	if(hadc->Instance == ADC1)
	{
		Measurement* meas = Measurement::GetInstance();
		AdcCallback(*meas, hadc);
	}
}

