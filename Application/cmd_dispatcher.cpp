#include "cmd_commands.hpp"
#include "cmd_defs.hpp"
#include "cmd_dispatcher.hpp"
#include "uartcom.hpp"
#include "measurement.hpp"

extern "C"
{
#include "gpio.h"
}

#if 0
const uint8_t CMD_ARG_OFFSET = 5u; //PA03_

if(lng != PWM_CMD_SIZE)
{
    return (uint16_t)(-1);
}

for(uint8_t idx = 0; idx < 3; idx++)
{
	if(pStrCmd[CMD_ARG_OFFSET+idx] > '9' || pStrCmd[CMD_ARG_OFFSET+idx] < '0')
	{
		return (uint16_t)(-1);
	}
}

uint32_t period = (pStrCmd[CMD_ARG_OFFSET + 0] - '0')*100;
period += (pStrCmd[CMD_ARG_OFFSET + 1] - '0')*10;
period += (pStrCmd[CMD_ARG_OFFSET + 2] - '0')*1;

if(period > 999)
{
    return (uint16_t)(-1);
}

//TIM2_CH1
if(memcmp(pStrCmd, "PA15", 4u) == 0)
{
	htim2.Instance->CCR1 = period;
	printf("PA15_%03lu\n", period);

}

//TIM2_CH2
if(memcmp(pStrCmd, "PB03", 4u) == 0)
{
	htim2.Instance->CCR2 = period;
	printf("PB03_%03lu\n", period);
}
#endif

uint32_t EnablePwmCh1(const uint8_t* const pStrCmd, const uint8_t lng)
{

}

uint32_t EnablePwmCh2(const uint8_t* const pStrCmd, const uint8_t lng)
{

}

uint32_t SetPwmCh1(const uint8_t* const pStrCmd, const uint8_t lng)
{

}

uint32_t SetPwmCh2(const uint8_t* const pStrCmd, const uint8_t lng)
{

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

constexpr uint CMD_TABLE_SIZE = 3u;

const CmdDisp_t cmdTable[CMD_TABLE_SIZE] = {

/*01*/    {method_write,     	cmd_blue_led,           	WriteBlueLED},
/*02*/    {method_read,     	cmd_temperature1,           ReadTemperature1},
/*03*/    {method_read,     	cmd_temperature2,    		ReadTemperature2}

};

uint32_t CommandDispatcher::Dispatch(const uint8_t* const pStrCmd, const uint8_t lng) {

    uint32_t result = CMD_RET_UKN;
    const char STR_CMD_OK[] = "CMD_OK\n";
    const char STR_CMD_ERR[] = "CMD_ERR\n";
    const char STR_CMD_UKN[] = "CMD_UKN\n";
    const char STR_CMD_FTL[] = "CMD_FTL\n";

    UartCom *uart = UartCom::GetInstance(UARTPeripheral::F7_UART3);

    for(uint8_t idx = 0; idx < CMD_TABLE_SIZE; idx++) {
        if ((!memcmp(pStrCmd, cmdTable[idx].method.word, CMD_METHOD_LNG)) &&
        !memcmp(pStrCmd + CMD_METHOD_LNG + CMD_DELIMITER_LNG, cmdTable[idx].command.word, CMD_NAME_LNG)) {

            result = cmdTable[idx].cmdFunc(pStrCmd, lng);
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
        }
        break;
        default : 
        {
            uart->Write(reinterpret_cast<uint8_t*>(const_cast<char*>(STR_CMD_FTL)), strlen(STR_CMD_FTL));
        }
    }

    return result;
}
