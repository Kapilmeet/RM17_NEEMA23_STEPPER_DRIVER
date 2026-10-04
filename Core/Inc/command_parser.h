#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include "stepper_control.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief  Initialise command parser state.
 */
void CMD_Init(void);

/**
 * @brief  Feed one byte from USB CDC into the parser.
 */
void CMD_ProcessByte(uint8_t byte, StepperMotor *motor);

/**
 * @brief  Periodic poll for the command parser.
 *         Automatically triggers execution if a command was sent without newline
 *         after a short timeout (300ms).
 */
void CMD_Poll(StepperMotor *motor);

/**
 * @brief  Reliable float-to-string formatter immune to newlib-nano float printf limitations.
 */
void CMD_FormatFloat(char *out, size_t out_sz, float val, uint8_t decimals);

#endif /* COMMAND_PARSER_H */
