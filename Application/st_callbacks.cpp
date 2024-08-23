/*
 * ST_Callbacks.c
 *
 *  Created on: Aug 17, 2024
 *      Author: 42077
 */

#include <st_callbacks.hpp>
#include <cassert>

void UartCallback(UartCom & uartcom, UART_HandleTypeDef *huart)
{

	    volatile uint8_t c = huart->Instance->RDR;

	    volatile uint8_t next_wr_idx = (uartcom.rx_buff_wr_idx + 1) & (UART_RX_BUFF_SIZE - 1);

	    if(next_wr_idx != uartcom.rx_buff_rd_idx && uartcom.rx_buff_overflow == 0)
	    {
	        if(c == '\n')
	        {
	        	uartcom.rx_buff_records++;
	        }

	        uartcom.rx_buff[uartcom.rx_buff_wr_idx] = c;

	        uartcom.rx_buff_wr_idx = next_wr_idx;
	    }
	    else
	    {
	    	uartcom.rx_buff[uartcom.rx_buff_wr_idx] = '\n';
	    	uartcom.rx_buff_records++;
	    	uartcom.rx_buff_overflow = 1;
	    }

	    assert(HAL_OK == HAL_UART_Receive_IT(uartcom.peripheral,
	    		const_cast<uint8_t*>(uartcom.rx_buff + uartcom.rx_buff_wr_idx),
				sizeof(uint8_t)));
}

void AdcCallback(Measurement & measurement, ADC_HandleTypeDef* hadc)
{
	/* Temperature sensor characteristics, RM0410 Reference manual,
	 * STM32F76xxx and STM32F77xxx ... , 15.10 (Page 464) */

	/*Basic formula: temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25

	float v_sense = (float)(adc[ADC_CH_RANK_2]*VDDA_NOM/ADC_MAX);
	temp[idx++] = ((v_sense - V25)/AVG_SLOPE) + T_OFFSET; */

	/* Measurement running for two ranks (two channel measured)*/
	measurement.FifoInsert(measurement.adc[ADC_CH_RANK_2]);

}
