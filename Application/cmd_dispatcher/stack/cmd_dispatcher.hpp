#ifndef CMD_DISPATCHER_HPP
#define CMD_DISPATCHER_HPP

#include "cmd_defs.hpp"

constexpr uint32_t SHARED_BUFF_SIZE = 128u;

class CommandDispatcher
{
public: 
    CommandDispatcher();
    ~CommandDispatcher() = default;
    uint32_t Dispatch(const uint8_t* const pStrCmd, const uint8_t lng);
    void HelpCommandPrintOut();
};

#endif // CMD_DISPATCHER_H_INCLUDED
