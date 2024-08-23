#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "cmd_dispatcher.hpp"
#include "uartcom.hpp"
#include "measurement.hpp"
#include "pwm.hpp"

extern "C"
{
#include "gpio.h"
}


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

	uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 0] - '0')*100;
	period += (pStrCmd[CMD_ARG_OFFSET + 1] - '0')*10;
	period += (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*1;

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

	uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 0] - '0')*100;
	period += (pStrCmd[CMD_ARG_OFFSET + 1] - '0')*10;
	period += (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*1;

	if(period > Pwm::PWM_MAX_PERIOD)
	{
	    return CMD_RET_ERR;
	}

    Pwm* pwm = Pwm::GetInstance();

    pwm->PwmSetPwmCh1(period);

    return CMD_RET_OK;
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

uint32_t ReadTemperature1(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG + CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	Measurement* measurement = Measurement::GetInstance();

	char buffer[8] = {0};

	uint32_t length = sprintf(buffer, "%+3.2f\n",  measurement->GetTemperature());

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    uart->Write(reinterpret_cast<uint8_t*>(buffer), length);

	return CMD_RET_OK;
}

uint32_t ReadTemperature2(const uint8_t* const pStrCmd, const uint8_t lng)
{
    if ((CMD_METHOD_LNG + CMD_NAME_LNG +
         CMD_DELIMITER_LNG + CMD_EOL_LNG) != lng) {

        return CMD_RET_ERR;
    }

	Measurement* measurement = Measurement::GetInstance();

	char buffer[8] = {0};

	uint32_t length = sprintf(buffer, "%+3.2f\n",  measurement->GetCalibTemperature());

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    uart->Write(reinterpret_cast<uint8_t*>(buffer), length);

	return CMD_RET_OK;
}

const CommandArg DEC_UI16_1 =
{
		DEC_UI16, 0, 1
};

const DispatcherCommand_t cmdWriteBlueLED =
{
		WriteBlueLED,
		DEC_UI16_1
};

const DispatcherCommand_t cmdWriteRedLED =
{
		WriteRedLED,
		DEC_UI16_1
};

const CmdDisp2_t testTable[2] = {
		{method_set,	cmd_blue_led,	cmdWriteBlueLED},
		{method_set,	cmd_red_led,	cmdWriteRedLED}
};

const argTypeLUT_t argTypeLUTtable[1] =
{
		{DEC_UI16, "DEC_UI16"}
};

const CmdDisp_t cmdTable[CMD_TABLE_SIZE] = {

/*01*/    {method_set,     		cmd_blue_led,           	WriteBlueLED},
/*02*/    {method_read,     	cmd_temperature1,           ReadTemperature1},
/*03*/    {method_read,     	cmd_temperature2,    		ReadTemperature2},
/*04*/    {method_enable,     	cmd_pwm1,    				EnablePwmCh1},
/*05*/    {method_enable,     	cmd_pwm2,    				EnablePwmCh2},
/*06*/    {method_set,     		cmd_pwm1,    				SetPwmCh1},
/*07*/    {method_set,     		cmd_pwm2,    				SetPwmCh1}

};

void HelpCommand(void)
{
    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

	for(uint32_t idx = 0; idx < 2u; idx++)
	{
		char helpLine[64] = {0};

		if(testTable[idx].cmdFunc.arg.type != NOARG)
		{
			uint32_t lng = sprintf(helpLine,
					"Typical: %s%c%s%c%lu\n",
					testTable[idx].method.word,
					CMD_DELIMITER,
					testTable[idx].command.word,
					CMD_DELIMITER,
					(testTable[idx].cmdFunc.arg.max + testTable[idx].cmdFunc.arg.min) >> 1
					);

			uart->Write(reinterpret_cast<uint8_t*>(helpLine), lng);

			memset(helpLine, 0u, 64);

			lng = sprintf(helpLine, "Arg. type: %s\nMin: %lu\nMax: %lu\n",
								argTypeLUTtable[testTable[idx].cmdFunc.arg.type].str,
								testTable[idx].cmdFunc.arg.min,
								testTable[idx].cmdFunc.arg.max
								);

			uart->Write(reinterpret_cast<uint8_t*>(helpLine), lng);
		}

	}
}

uint32_t CommandDispatcher::Dispatch(const uint8_t* const pStrCmd, const uint8_t lng) {

    uint32_t result = CMD_RET_UKN;
    const char STR_CMD_OK[] = "CMD_OK\n";
    const char STR_CMD_ERR[] = "CMD_ERR\n";
    const char STR_CMD_UKN[] = "CMD_UKN\n";
    const char STR_CMD_FTL[] = "CMD_FTL\n";

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);
/*
    for(uint8_t idx = 0; idx < CMD_TABLE_SIZE; idx++) {
        if ((!memcmp(pStrCmd, cmdTable[idx].method.word, CMD_METHOD_LNG)) &&
        !memcmp(pStrCmd + CMD_METHOD_LNG + CMD_DELIMITER_LNG, cmdTable[idx].command.word, CMD_NAME_LNG)) {

            result = cmdTable[idx].cmdFunc(pStrCmd, lng);
            break;
        }
    }
*/
    for(uint8_t idx = 0; idx < 2u; idx++) {
        if ((!memcmp(pStrCmd, testTable[idx].method.word, CMD_METHOD_LNG)) &&
        !memcmp(pStrCmd + CMD_METHOD_LNG + CMD_DELIMITER_LNG, testTable[idx].command.word, CMD_NAME_LNG)) {

            result = testTable[idx].cmdFunc.function(pStrCmd, lng);
            break;
        }
    }

    switch(result)
    {
        case CMD_RET_OK : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_OK)), strlen(STR_CMD_OK));
        }
        break;
        case CMD_RET_ERR : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_ERR)), strlen(STR_CMD_ERR));
        }
        break;
        case CMD_RET_UKN : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_UKN)), strlen(STR_CMD_UKN));
            HelpCommand();
        }
        break;
        default : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_FTL)), strlen(STR_CMD_FTL));
        }
    }

    return result;
}
