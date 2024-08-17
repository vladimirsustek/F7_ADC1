#ifndef CMD_DISPATCHER_HPP
#define CMD_DISPATCHER_HPP

#include "cmd_defs.hpp"

class CommandDispatcher
{
public: 
    CommandDispatcher() = default;
    ~CommandDispatcher() = default;
    uint32_t Dispatch(const uint8_t* const pStrCmd, const uint8_t lng);
};

#endif // CMD_DISPATCHER_H_INCLUDED
