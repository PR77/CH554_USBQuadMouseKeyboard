/********************************** (C) COPYRIGHT *******************************
* File Name     : SERIAL.H
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2025/02/05
* Description   : 8051 UART Common
*******************************************************************************/

#ifndef __CH554_SERIAL_H__
#define __CH554_SERIAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "ch554.h"
#include "serial_0.h"
#include "serial_1.h"

#define SERIAL_CONSOLE_COLUMN_WIDTH     80

void serial_printCharacter(char character);
uint16_t serial_getCharacter(uint32_t timeout);
void serial_printStringPadded(char *string, uint8_t stringPaddingSize);
void serial_printStringTitle(char *string, uint8_t totalLineLength, char paddingCharacter, bool addNewLine);
void serial_printString(char* string);
void serial_printHexByte(uint8_t value);
void serial_printHexWord(uint16_t value);

#endif // __CH554_SERIAL_H__