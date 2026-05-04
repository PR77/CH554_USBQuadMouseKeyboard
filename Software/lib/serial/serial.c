/********************************** (C) COPYRIGHT *******************************
* File Name     : SERIAL.C
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2025/02/06
* Description   : 8051 UART Common
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <compiler.h>
#include <string.h>
#include <stdlib.h>
#include "ch554.h"
#include "serial_0.h"
#include "serial_1.h"
#include "serial_cfg.h"

// HEX encoding table
static const uint8_t hexTable[] = {
    '0', '1', '2', '3',
    '4', '5', '6', '7',
    '8', '9', 'A', 'B',
    'C', 'D', 'E', 'F'
};

void serial_printCharacter(char character) {
    CONSOLE_PORT_PUTCHR(character);
}

uint16_t serial_getCharacter(uint32_t timeout) {
    return (CONSOLE_PORT_GETCHR(timeout));
}

void serial_printStringPadded(char *string, uint8_t stringPaddingSize) {
    
    uint8_t stringIndex = 0;

    if (string != NULL) {
        while (string[stringIndex] != '\0') {
            // Repeat until string terminator reached printing each character
            // on the console.
            CONSOLE_PORT_PUTCHR(string[stringIndex]);
            stringIndex++;
        }
    }

    // If string == NULL still do padding to ensure user interface consistency.
    while (stringIndex < stringPaddingSize) {
        // Now if there are still character to print but the padding number
        // has been reached, just print 'SPACE' characters.
        CONSOLE_PORT_PUTCHR(' ');
        stringIndex++;
    }
}

void serial_printStringTitle(char *string, uint8_t totalLineLength, char paddingCharacter, bool addNewLine) {

    uint8_t numberOfPadCharacters = 0;
    uint8_t stringLength = strlen(string);
    
    if (string != NULL) {
        if (stringLength < totalLineLength) {
            // Determine number of needed padding characters. If string is NULL, then
            // fill whole line with the padding character.
            numberOfPadCharacters = totalLineLength - stringLength - 1;
        }
        
        while (*string) {
            // Repeat until string terminator reached printing each character
            // on the console.
            CONSOLE_PORT_PUTCHR(*string++);
        }
        
        CONSOLE_PORT_PUTCHR(' ');
    } else {
        numberOfPadCharacters = totalLineLength;
    }
    
    for (uint8_t i = 0; i < numberOfPadCharacters; i++) {
        CONSOLE_PORT_PUTCHR(paddingCharacter);
    }

    if (addNewLine) {
        CONSOLE_PORT_PUTCHR('\n'); 
    }
}

void serial_printString(char* string) {

    if (string == NULL) {
        return;
    }
    
    while (*string) {
        // Repeat until string terminator reached printing each character
        // on the console.
        CONSOLE_PORT_PUTCHR(*string++);
    }
}

void serial_printHexByte(uint8_t value) {
    CONSOLE_PORT_PUTCHR(hexTable[(value >> 4)]);
    CONSOLE_PORT_PUTCHR(hexTable[value & 0x0F]);
}

void serial_printHexWord(uint16_t value) {
    serial_printHexByte(value >> 8);
    serial_printHexByte(value & 0xFF);
}
