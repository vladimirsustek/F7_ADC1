#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "cmd_dispatcher.hpp"
#include "uartcom.hpp"
#include "measurement.hpp"
#include "pwm.hpp"

#include "cmd_pwm.hpp"
#include "cmd_gpio.hpp"
#include "cmd_meas.hpp"

constexpr uint32_t HELP_MAX_LINE_LNG = 64u;

const CmdDisp_t cmdTable[CMD_TABLE_SIZE] = {
/*01*/		{method_set,		cmd_blue_led,			cmdWriteBlueLED},
/*02*/		{method_set,		cmd_red_led,			cmdWriteRedLED},
/*03*/		{method_read,		cmd_temperature1,		cmdReadTemperature1},
/*04*/		{method_read,		cmd_temperature2,		cmdReadTemperature2},
/*05*/		{method_enable,		cmd_pwm1,				cmdEnablePwmCh1},
/*06*/		{method_enable,		cmd_pwm2,				cmdEnablePwmCh2},
/*07*/		{method_set,		cmd_pwm1,				cmdSetPwmCh1},
/*08*/		{method_set,		cmd_pwm2,				cmdSetPwmCh2}
};

void HelpCommandPrintOut(void)
{
    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);
	char helpLine[HELP_MAX_LINE_LNG] = {0};
    uint32_t lng;

	for(uint32_t idx = 0; idx < CMD_TABLE_SIZE; idx++)
	{
		if(cmdTable[idx].cmdFunc.arg.type != NOARG)
		{
			lng = sprintf(helpLine,
					"----------------------\n"
					"Typical: %s%c%s%c%lu\n",
					cmdTable[idx].method.word,
					CMD_DELIMITER,
					cmdTable[idx].command.word,
					CMD_DELIMITER,
					(cmdTable[idx].cmdFunc.arg.max + cmdTable[idx].cmdFunc.arg.min) >> 1
					);

			assert(lng <= HELP_MAX_LINE_LNG);

			uart->Write(reinterpret_cast<uint8_t*>(helpLine), lng);

			memset(helpLine, 0u, 64);

			lng = sprintf(helpLine, "Arg. type: %s\nMin: %lu\nMax: %lu\n",
								argTypeLUTtable[cmdTable[idx].cmdFunc.arg.type].str,
								cmdTable[idx].cmdFunc.arg.min,
								cmdTable[idx].cmdFunc.arg.max
								);

			assert(lng <= HELP_MAX_LINE_LNG);

			uart->Write(reinterpret_cast<uint8_t*>(helpLine), lng);
		}
		else
		{
			lng = sprintf(helpLine,
					"----------------------\n"
					"Only : %s%c%s\n",
					cmdTable[idx].method.word,
					CMD_DELIMITER,
					cmdTable[idx].command.word
					);

			assert(lng <= HELP_MAX_LINE_LNG);

			uart->Write(reinterpret_cast<uint8_t*>(helpLine), lng);

		}
	}
}

uint32_t CommandDispatcher::Dispatch(const uint8_t* const pStrCmd, const uint8_t lng) {

    uint32_t result = CMD_RET_UKN;
    const char STR_CMD_OK[] = "CMD_OK\n";
    const char STR_CMD_ERR[] = "CMD_ERR\n";
    const char STR_CMD_UKN[] = "CMD_UKN -> Try: \"HELP\\n\"\n";
    const char STR_CMD_FTL[] = "CMD_FTL\n";

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    for(uint8_t idx = 0; idx < CMD_TABLE_SIZE; idx++) {
        if ((!memcmp(pStrCmd, cmdTable[idx].method.word, CMD_METHOD_LNG)) &&
        !memcmp(pStrCmd + CMD_METHOD_LNG + CMD_DELIMITER_LNG, cmdTable[idx].command.word, CMD_NAME_LNG)) {

            result = cmdTable[idx].cmdFunc.function(pStrCmd, lng);
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
            if(!memcmp(pStrCmd, "HELP\n", strlen("HELP\n")))
            {
            	HelpCommandPrintOut();
            }
            else
            {
                uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_UKN)),
                		strlen(STR_CMD_UKN));
            }
        }
        break;
        default : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_FTL)), strlen(STR_CMD_FTL));
        }
    }

    return result;
}
