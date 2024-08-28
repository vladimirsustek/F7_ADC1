#ifndef CMD_COMMANDS_HPP
#define CMD_COMMANDS_HPP

#include "cmd_defs.hpp"

/*** BASIC METHODS ***/
const METHOD method_read = METHOD("RD");
const METHOD method_write = METHOD("WR");
const METHOD method_enable = METHOD("EN");
const METHOD method_set = METHOD("ST");

/** COMMANDS */
const COMMAND cmd_temperature1 = COMMAND("TER1");
const COMMAND cmd_temperature2 = COMMAND("TER2");
const COMMAND cmd_blue_led = COMMAND("LEDB");
const COMMAND cmd_red_led = COMMAND("LEDR");
const COMMAND cmd_pwm1 = COMMAND("PWM1");
const COMMAND cmd_pwm2 = COMMAND("PWM2");

#endif // CMD_COMMANDS_H_INCLUDED
