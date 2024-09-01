#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "cmd_dispatcher.hpp"
#include "cmd_table.hpp"
#include "cmd_ioport.hpp"


CommandDispatcher::CommandDispatcher()
{
	cmdDispOutInit();
}


void CommandDispatcher::HelpCommandPrintOut()
{
	char helpLine[HELP_MAX_LINE_LNG];

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

			cmdDispOutMsg(helpLine, lng);

			memset(helpLine, 0u, 64);

			lng = sprintf(helpLine, "Arg. type: %s\nMin: %lu\nMax: %lu\n",
								argTypeLUTtable[cmdTable[idx].cmdFunc.arg.type].str,
								cmdTable[idx].cmdFunc.arg.min,
								cmdTable[idx].cmdFunc.arg.max
								);

			assert(lng <= HELP_MAX_LINE_LNG);

			cmdDispOutMsg(helpLine, lng);
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

			cmdDispOutMsg(helpLine, lng);

		}
	}
}

uint32_t CommandDispatcher::Dispatch(const uint8_t* const pStrCmd, const uint8_t lng) {

    uint32_t result = CMD_RET_UKN;
    const char STR_CMD_OK[] = "CMD_OK\n";
    const char STR_CMD_ERR[] = "CMD_ERR\n";
    const char STR_CMD_UKN[] = "CMD_UKN -> Try: \"HELP\\n\"\n";
    const char STR_CMD_FTL[] = "CMD_FTL\n";

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
        	cmdDispOutMsg(STR_CMD_OK, strlen(STR_CMD_OK));
        }
        break;
        case CMD_RET_ERR : 
        {
        	cmdDispOutMsg(STR_CMD_ERR, strlen(STR_CMD_ERR));
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
            	cmdDispOutMsg(STR_CMD_UKN, strlen(STR_CMD_UKN));
            }
        }
        break;
        default : 
        {
        	cmdDispOutMsg(STR_CMD_FTL, strlen(STR_CMD_FTL));
        }
    }

    return result;
}
