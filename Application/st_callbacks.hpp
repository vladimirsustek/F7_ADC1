/*
 * ST_Callbacks.h
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */

#ifndef ST_CALLBACKS_HPP_
#define ST_CALLBACKS_HPP_

#include "uartcom.hpp"
#include "measurement.hpp"

void UartCallback(UartCom & uartcom, UART_HandleTypeDef *huart);
void AdcCallback(Measurement & measurement, ADC_HandleTypeDef* hadc);

#endif /* ST_CALLBACKS_HPP_ */
