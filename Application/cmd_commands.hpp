#ifndef CMD_COMMANDS_HPP
#define CMD_COMMANDS_HPP

#include "cmd_defs.hpp"

/*** BASIC METHODS ***/
const METHOD method_read = METHOD("RD");
const METHOD method_write = METHOD("WR");
const METHOD method_enable = METHOD("EN");

/** COMMANDS */
const COMMAND cmd_temperature1 = COMMAND("TER1");
const COMMAND cmd_temperature2 = COMMAND("TER2");
const COMMAND cmd_blue_led = COMMAND("LEDB");

#endif // CMD_COMMANDS_H_INCLUDED
