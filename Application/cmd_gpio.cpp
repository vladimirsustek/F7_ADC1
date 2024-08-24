/*
 * cmd_gpio.cpp
 *
 *  Created on: Aug 23, 2024
 *      Author: 42077
 */

#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "pwm.hpp"

extern "C"
{
#include "gpio.h"
}

uint32_t WriteBlueLED(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG1_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

    GPIO_PinState enable_arg = static_cast<GPIO_PinState>(
    		(pStrCmd[CMD_ARG_OFFSET] - '0')*1);
    if(enable_arg != 0u && enable_arg != 1u)
    {
        return CMD_RET_ERR;
    }

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, enable_arg);

    return 0;
}

uint32_t WriteRedLED(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG1_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

    GPIO_PinState enable_arg = static_cast<GPIO_PinState>(
    		(pStrCmd[CMD_ARG_OFFSET] - '0')*1);
    if(enable_arg != 0u && enable_arg != 1u)
    {
        return CMD_RET_ERR;
    }

    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, enable_arg);

    return 0;
}
