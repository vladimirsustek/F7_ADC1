/*
 * cmd_utils.cpp
 *
 *  Created on: Sep 1, 2024
 *      Author: 42077
 */

#include "cmd_utils.hpp"

uint32_t ui32toStr(uint32_t num, char *buffer, uint32_t buff_size)
{

    uint32_t num_to_process = num;
    uint32_t total_digits;
    uint32_t digits = 0;

    while(num_to_process)
    {
        num_to_process /= 10;
        digits++;
    }

    total_digits = digits;

    if (buffer == nullptr || buff_size < digits)
    {
        return (uint32_t)(-1);
    }

    num_to_process = num;

    while(num)
    {
        buffer[digits-1] = (num % 10) + '0';
        num /= 10;
        digits--;
    }

    return total_digits;
}

uint32_t i32toStr(int32_t num, char *buffer, uint32_t buff_size)
{
	uint32_t digits = 0;
    if(num < 0)
    {
    	digits = ui32toStr(-1*num, buffer+1, buff_size);
        buffer[0] = '-';
        digits += 1;
    }
    else
    {
    	digits = ui32toStr(num, buffer, buff_size);
    }

    return digits;
}
