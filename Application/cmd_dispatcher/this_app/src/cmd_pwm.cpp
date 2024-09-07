/*
 * cmd_pwm.cpp
 *
 *  Created on: Aug 23, 2024
 *      Author: 42077
 */


#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "pwm.hpp"

uint32_t EnablePwmCh1(const uint8_t* const pStrCmd, const uint8_t lng)
{

    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG1_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

    uint32_t enable_arg = static_cast<uint32_t>(
    		(pStrCmd[CMD_ARG_OFFSET] - '0')*1);
    if(enable_arg != 0u && enable_arg != 1u)
    {
        return CMD_RET_ERR;
    }

    Pwm* pwm = Pwm::GetInstance();

    if(enable_arg)
    {
    	pwm->PwmStartCh1();
    }
    else
    {
    	pwm->PwmStopCh1();
    }

    return CMD_RET_OK;
}

uint32_t EnablePwmCh2(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG1_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

    uint32_t enable_arg = static_cast<uint32_t>(
    		(pStrCmd[CMD_ARG_OFFSET] - '0')*1);
    if(enable_arg != 0u && enable_arg != 1u)
    {
        return CMD_RET_ERR;
    }

    Pwm* pwm = Pwm::GetInstance();

    if(enable_arg)
    {
    	pwm->PwmStartCh2();
    }
    else
    {
    	pwm->PwmStopCh2();
    }

    return CMD_RET_OK;
}

uint32_t SetPwmCh1(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG5_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	for(uint8_t idx = 0; idx < 3; idx++)
	{
		if(pStrCmd[CMD_ARG_OFFSET+idx] > '9' || pStrCmd[CMD_ARG_OFFSET+idx] < '0')
		{
			return CMD_RET_ERR;
		}
	}

	uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*100;
	period += (pStrCmd[CMD_ARG_OFFSET + 3] - '0')*10;
	period += (pStrCmd[CMD_ARG_OFFSET + 4] - '0')*1;

	if(period > Pwm::PWM_MAX_PERIOD)
	{
	    return CMD_RET_ERR;
	}

    Pwm* pwm = Pwm::GetInstance();

    pwm->PwmSetPwmCh1(period);

    return CMD_RET_OK;
}

uint32_t SetPwmCh2(const uint8_t* const pStrCmd, const uint8_t lng)
{

    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG*2 + CMD_ARG5_LNG+
         CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	for(uint8_t idx = 0; idx < 3; idx++)
	{
		if(pStrCmd[CMD_ARG_OFFSET+idx] > '9' || pStrCmd[CMD_ARG_OFFSET+idx] < '0')
		{
			return CMD_RET_ERR;
		}
	}

	uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*100;
	period += (pStrCmd[CMD_ARG_OFFSET + 3] - '0')*10;
	period += (pStrCmd[CMD_ARG_OFFSET + 4] - '0')*1;

	if(period > Pwm::PWM_MAX_PERIOD)
	{
	    return CMD_RET_ERR;
	}

    Pwm* pwm = Pwm::GetInstance();

    pwm->PwmSetPwmCh1(period);

    return CMD_RET_OK;
}

