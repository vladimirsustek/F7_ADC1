#ifndef CMD_DEFS_HPP
#define CMD_DEFS_HPP

extern "C"
{
    #include "string.h"
    #include "stdio.h"
    #include "stdint.h"
}


constexpr uint32_t CMD_EOL_LNG=1u;
constexpr uint32_t CMD_DELIMITER_LNG=1u;
constexpr char CMD_DELIMITER='_';

constexpr uint32_t CMD_PLATFORM_LNG	=0u;

constexpr uint32_t CMD_METHOD_LNG=2u;
constexpr uint32_t CMD_NAME_LNG=4u;

constexpr uint32_t CMD_ARG1_LNG=1u;
constexpr uint32_t CMD_ARG2_LNG=2u;
constexpr uint32_t CMD_ARG5_LNG=5u;

constexpr uint32_t CMD_ARG_OFFSET=(CMD_PLATFORM_LNG + CMD_NAME_LNG + CMD_METHOD_LNG + CMD_DELIMITER_LNG*2);

constexpr uint32_t CMD_RET_OK =0x0;
constexpr uint32_t CMD_RET_ERR=0xFFFF;
constexpr uint32_t CMD_RET_UKN=0xF0F0;

typedef uint32_t (*pfn_u32_cpuc8cu8)(const uint8_t* const, const uint8_t lng);

class METHOD
{
public:
    METHOD(const char* p) {strcpy(const_cast<char*>(word), const_cast<char*>(p));}
    char word[3];
};

class COMMAND
{
public:
    COMMAND(const char* p) {strcpy(const_cast<char*>(word), const_cast<char*>(p));}
    char word[4];
};

typedef struct CmdDisp{
    METHOD method;
    COMMAND command;
    pfn_u32_cpuc8cu8 cmdFunc;
} CmdDisp_t;



#endif // CMD_DEFS_H_INCLUDED
