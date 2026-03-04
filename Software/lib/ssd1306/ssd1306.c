/********************************** (C) COPYRIGHT *******************************
* File Name     : SSD1306.C
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2024/12/17
* Description   : SSD1306 Driver
*******************************************************************************/

#include <stdint.h>
#include <compiler.h>
#include "ch554.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_font.h"

// References - 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/

// Select the screen size
#define SCREEN_128x32

#if defined(SCREEN_128x32) && defined(SCREEN_128x64)
#error "Please define either SCREEN_128x32 or SCREEN_128x64 but not both!"
#endif

#if !defined(SCREEN_128x32) && !defined(SCREEN_128x64)
#error "Please define one of SCREEN_128x32 or SCREEN_128x64!"
#endif

#if defined(SCREEN_128x32)
#define PAGES               4
#define MULTIPLE            1
#else
#define PAGES               8
#define MULTIPLE            2
#endif

#define OLED_FONT_WIDTH     5
#define OLED_FONT_HEIGHT    8

// -----------------------------------------------------------------------------
// OLED Implementation
// -----------------------------------------------------------------------------

// OLED init settings
static const uint8_t OLED_INIT_CMD[] = {
    OLED_MULTIPLEX, ((PAGES * 8) - 1),  // set multiplex (HEIGHT-1): 31 for 128x32, 63 for 128x64
#if defined(SCREEN_128x32)  
    OLED_MEMORYMODE, 0x00,          // set page memory addressing mode
    OLED_PAGES, 0x00, (PAGES - 1),  // set min and max page
    OLED_COMPINS, 0x02,             // set COM pins hardware configuration to sequential
#endif  
    OLED_CHARGEPUMP, 0x14,          // enable charge pump
    OLED_CONTRAST, 0x4F,            // set contrast to middle value
    OLED_VCOMH_LEVEL, 0x40,         // VCOMH setting
    OLED_PRE_CHARGE, 0xF1,          // pre-charge period
    OLED_DISPLAY_ON,                // switch on OLED
    OLED_XFLIP_OFF, OLED_YFLIP_OFF  // flip the screen
};

// HEX encoding table
static const uint8_t hexTable[] = {
    '0', '1', '2', '3',
    '4', '5', '6', '7',
    '8', '9', 'A', 'B',
    'C', 'D', 'E', 'F'
};

// OLED init function
void ssd1306_initialise(void) {
    i2c_startCommunication(OLED_ADDR);
    i2c_writeByte(OLED_CMD_MODE);
    for (uint8_t i = 0; i < sizeof(OLED_INIT_CMD); i++) {
        i2c_writeByte(OLED_INIT_CMD[i]);
    }
    i2c_stopCommunication();
}

// OLED set vertical shift
void ssd1306_verticalShift(uint8_t yPosition) {
    i2c_startCommunication(OLED_ADDR);
    i2c_writeByte(OLED_CMD_MODE);
    i2c_writeByte(OLED_OFFSET);
    i2c_writeByte(yPosition);
    i2c_stopCommunication();
}

// OLED draw BMP image
void ssd1306_drawBmp(uint8_t xPosition, uint8_t yPosition, uint8_t xSize, uint8_t ySize, const uint8_t* bmpData) {

    uint8_t startPage = yPosition >> 3;                                 // each pages is 8 pixels "high"
    uint8_t numberOfPages = ySize >> 3;

    for (uint8_t pageLoop = 0; pageLoop < numberOfPages; pageLoop++) {
        
        // Set page number based on loop itteration and starting x position
        i2c_startCommunication(OLED_ADDR);
        i2c_writeByte(OLED_CMD_MODE);
        i2c_writeByte(OLED_PAGE | ((startPage + pageLoop) & 0x07));
        i2c_writeByte(OLED_COLUMN_LOW | (xPosition & 0x0F));            // set low nibble of start column
        i2c_writeByte(OLED_COLUMN_HIGH | (xPosition >> 4));             // set high nibble of start column
        i2c_stopCommunication();

        // Write bitmap data
        i2c_startCommunication(OLED_ADDR);
        i2c_writeByte(OLED_DAT_MODE);    

        for (uint8_t i = 0; i < xSize; i++) {
            i2c_writeByte(*bmpData++);
        }
     
        i2c_stopCommunication();
    }
}

// OLED print a character
void ssd1306_printCharacter(char character) {
    uint16_t offset = 0;

    if (character >= 32) {
        offset = character - 32;
    } else {
        return;
    }

    offset += offset << 2;                  // -> offset = (ch - 32) * 5
    i2c_startCommunication(OLED_ADDR);
    i2c_writeByte(OLED_DAT_MODE);    
    i2c_writeByte(0x00);                    // print spacing between characters
    
    for(uint8_t i = 0; i < OLED_FONT_WIDTH; i++) {
        i2c_writeByte(OLED_FONT[offset++]);
    }
    i2c_stopCommunication();
}

// OLED print a string from program memory
void ssd1306_printString(char* string) {
    
    if (!string) {
        return;
    }

    while (*string) {                       // repeat until string terminator
        ssd1306_printCharacter(*string++);  // print character on OLED
    }
}

void ssd1306_printHexByte(uint8_t value) {
    ssd1306_printCharacter(hexTable[(value >> 4)]);
    ssd1306_printCharacter(hexTable[value & 0x0F]);
}

void ssd1306_printHexWord(uint16_t value) {
    ssd1306_printHexByte(value >> 8);
    ssd1306_printHexByte(value & 0xFF);
}

// OLED set the cursor
void ssd1306_setCursor(uint8_t xPosition, uint8_t yPosition) {
    uint8_t xPositionCharacter = xPosition * (OLED_FONT_WIDTH + 1); // x postion based on character width

    i2c_startCommunication(OLED_ADDR);
    i2c_writeByte(OLED_CMD_MODE);
    i2c_writeByte(OLED_PAGE | (yPosition & 0x07));                  // set start page
    i2c_writeByte(OLED_COLUMN_LOW | (xPositionCharacter & 0x0F));   // set low nibble of start column
    i2c_writeByte(OLED_COLUMN_HIGH | (xPositionCharacter >> 4));    // set high nibble of start column
    i2c_stopCommunication();
}

// OLED clear screen
void ssd1306_clearScreen(void) {

    for (uint8_t i = 0; i < PAGES; i++) {       // clear screen line by line
        ssd1306_setCursor(0, i);
        i2c_startCommunication(OLED_ADDR);      // start transmission to OLED
        i2c_writeByte(OLED_DAT_MODE);           // set data mode
        
        for(uint8_t i = 128; i; i--) {
            i2c_writeByte(0x00);
        }
        i2c_stopCommunication();                // stop transmission
    }
}

void ssd1306_test(void) {
    // print all characters on 4 lines, 20 per line
    uint8_t c = 32;

    for (uint8_t l = 0; l < 4; l++) {
        ssd1306_setCursor(0, l * MULTIPLE);
        i2c_startCommunication(OLED_ADDR);
        i2c_writeByte(OLED_DAT_MODE);
        
        for (uint8_t p = 20; p; p--) {
            ssd1306_printCharacter(c++);
            if (c == 32 + 64) {
                break;
            }
        }
        i2c_stopCommunication();
    }
}