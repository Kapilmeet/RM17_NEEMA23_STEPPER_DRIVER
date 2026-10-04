/**
 * @file    command_parser.c
 * @brief   USB CDC Command Parser for Stepper Motion Controller
 *
 * Supported Command Formats:
 *   <angle>, <ppr> [, <accel>] [, <speed>]
 *   G <angle>, <ppr> [, <accel>] [, <speed>]
 *   P or POS?       -> Query current output angle
 *   H or HOME       -> Set current position to 0.0 degrees
 *   S or STOP       -> Emergency stop
 *   ? or STATUS     -> Query motor status
 */

#include "command_parser.h"
#include "stepper_config.h"
#include "usb_cdc.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>

#define CMD_BUFFER_SIZE 128

static char     cmd_buffer[CMD_BUFFER_SIZE];
static uint16_t cmd_index = 0;
static uint32_t last_byte_time = 0;

void CMD_Init(void)
{
    cmd_index = 0;
    cmd_buffer[0] = '\0';
    last_byte_time = 0;
}

static void trim_whitespace(char *str)
{
    char *end;
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
}

static void execute_command(char *line, StepperMotor *motor)
{
    char reply[128];

    /* Trim leading and trailing whitespace */
    trim_whitespace(line);
    if (strlen(line) == 0) {
        return;
    }

    /* 1. Query Position: "P" or "POS" or "POS?" */
    if (strcasecmp(line, "P") == 0 ||
        strcasecmp(line, "POS") == 0 ||
        strcasecmp(line, "POS?") == 0)
    {
        snprintf(reply, sizeof(reply), "POS: %.3f DEG\r\n",
                 Stepper_GetPosition(motor));
        USB_CDC_Print(reply);
        return;
    }

    /* 2. Set Home / Zero: "H" or "HOME" or "ZERO" */
    if (strcasecmp(line, "H") == 0 ||
        strcasecmp(line, "HOME") == 0 ||
        strcasecmp(line, "ZERO") == 0)
    {
        Stepper_SetHome(motor);
        USB_CDC_Print("HOME: Position reset to 0.0 DEG\r\n");
        return;
    }

    /* 3. Emergency Stop: "S" or "STOP" */
    if (strcasecmp(line, "S") == 0 || strcasecmp(line, "STOP") == 0) {
        Stepper_Stop(motor);
        snprintf(reply, sizeof(reply), "STOP: Motor halted at %.3f DEG\r\n",
                 Stepper_GetPosition(motor));
        USB_CDC_Print(reply);
        return;
    }

    /* 4. Status Query: "?" or "STATUS" */
    if (strcmp(line, "?") == 0 || strcasecmp(line, "STATUS") == 0) {
        snprintf(reply, sizeof(reply),
                 "STATUS: Pos=%.3f DEG | Moving=%d | State=%d | PPR=%lu\r\n",
                 Stepper_GetPosition(motor), motor->moving, motor->state,
                 (unsigned long)motor->pulses_per_rev);
        USB_CDC_Print(reply);
        return;
    }

    /* 5. Move Command:
     *    Format: [G/MOVE] <angle>, <ppr> [, <accel>] [, <speed>]
     *    Optional fields can be omitted or left blank.
     */
    char *ptr = line;

    /* Skip optional 'G' or 'MOVE' prefix */
    if (*ptr == 'G' || *ptr == 'g') {
        ptr++;
    } else if (strncasecmp(ptr, "MOVE", 4) == 0) {
        ptr += 4;
    }
    while (isspace((unsigned char)*ptr)) ptr++;

    /* If motor is currently executing a move, reject new move */
    if (motor->moving) {
        USB_CDC_Print("BUSY: Motor is currently moving\r\n");
        return;
    }

    /* Tokenize by commas or spaces */
    char *token1 = NULL;
    char *token2 = NULL;
    char *token3 = NULL;
    char *token4 = NULL;

    /* Check if line contains commas */
    if (strchr(ptr, ',') != NULL) {
        token1 = ptr;
        char *c1 = strchr(token1, ',');
        if (c1) {
            *c1 = '\0';
            token2 = c1 + 1;
            char *c2 = strchr(token2, ',');
            if (c2) {
                *c2 = '\0';
                token3 = c2 + 1;
                char *c3 = strchr(token3, ',');
                if (c3) {
                    *c3 = '\0';
                    token4 = c3 + 1;
                }
            }
        }
    } else {
        /* Space-separated tokens */
        token1 = strtok(ptr, " \t");
        token2 = strtok(NULL, " \t");
        token3 = strtok(NULL, " \t");
        token4 = strtok(NULL, " \t");
    }

    if (!token1 || !token2) {
        USB_CDC_Print("ERR: Expected <angle>, <ppr> [, <accel>] [, <speed>]\r\n");
        return;
    }

    trim_whitespace(token1);
    trim_whitespace(token2);
    if (token3) trim_whitespace(token3);
    if (token4) trim_whitespace(token4);

    /* Mandatory parameters: Angle (deg) and Microstepping PPR */
    char *endptr;
    float target_angle = strtof(token1, &endptr);
    if (endptr == token1) {
        USB_CDC_Print("ERR: Invalid angle value\r\n");
        return;
    }

    uint32_t ppr = (uint32_t)strtoul(token2, &endptr, 10);
    if (endptr == token2 || ppr == 0) {
        USB_CDC_Print("ERR: Invalid PPR value (must be > 0)\r\n");
        return;
    }

    /* Optional parameters: Output Accel (rad/s^2) and Output Speed (rad/s) */
    float accel = DEFAULT_OUTPUT_ACCEL;
    if (token3 && strlen(token3) > 0) {
        float parsed_accel = strtof(token3, &endptr);
        if (endptr != token3 && parsed_accel > 0.0f) {
            accel = parsed_accel;
        }
    }

    float speed = DEFAULT_OUTPUT_SPEED;
    if (token4 && strlen(token4) > 0) {
        float parsed_speed = strtof(token4, &endptr);
        if (endptr != token4 && parsed_speed > 0.0f) {
            speed = parsed_speed;
        }
    }

    /* Execute the move */
    Stepper_MoveTo(motor, target_angle, ppr, accel, speed);

    snprintf(reply, sizeof(reply),
             "OK: Moving to %.3f DEG | PPR=%lu | Accel=%.2f rad/s2 | Speed=%.2f rad/s\r\n",
             target_angle, (unsigned long)ppr, accel, speed);
    USB_CDC_Print(reply);
}

void CMD_ProcessByte(uint8_t byte, StepperMotor *motor)
{
    last_byte_time = HAL_GetTick();

    /* End of command line */
    if (byte == '\r' || byte == '\n') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';
            execute_command(cmd_buffer, motor);
            cmd_index = 0;
        }
    } else if (byte == 0x08 || byte == 0x7F) {
        /* Backspace support */
        if (cmd_index > 0) {
            cmd_index--;
        }
    } else {
        /* Append character to line buffer */
        if (cmd_index < (CMD_BUFFER_SIZE - 1)) {
            cmd_buffer[cmd_index++] = (char)byte;
        }
    }
}

void CMD_Poll(StepperMotor *motor)
{
    /* If characters have been in the buffer for > 250ms without \r or \n, auto-execute */
    if (cmd_index > 0 && (HAL_GetTick() - last_byte_time) > 250) {
        cmd_buffer[cmd_index] = '\0';
        execute_command(cmd_buffer, motor);
        cmd_index = 0;
    }
}
