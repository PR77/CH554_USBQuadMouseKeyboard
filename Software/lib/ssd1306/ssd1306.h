/********************************** (C) COPYRIGHT *******************************
* File Name     : SSD1306.H
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2024/12/17
* Description   : SSD1306 Driver
*******************************************************************************/

#ifndef __CH554_SSD1306_H__
#define __CH554_SSD1306_H__

#include <stdint.h>
#include "ch554.h"

// OLED definitions
#define OLED_ADDR         0x78    // OLED write address (0x3C << 1)
#define OLED_CMD_MODE     0x00    // set command mode
#define OLED_DAT_MODE     0x40    // set data mode

// OLED commands
#define OLED_COLUMN_LOW   0x00    // set lower 4 bits of start column (0x00 - 0x0F)
#define OLED_COLUMN_HIGH  0x10    // set higher 4 bits of start column (0x10 - 0x1F)
#define OLED_MEMORYMODE   0x20    // set memory addressing mode (following byte)
#define OLED_COLUMNS      0x21    // set start and end column (following 2 bytes)
#define OLED_PAGES        0x22    // set start and end page (following 2 bytes)
#define OLED_STARTLINE    0x40    // set display start line (0x40-0x7F = 0-63)
#define OLED_CONTRAST     0x81    // set display contrast (following byte)
#define OLED_CHARGEPUMP   0x8D    // (following byte - 0x14:enable, 0x10: disable)
#define OLED_XFLIP_OFF    0xA0    // don't flip display horizontally
#define OLED_XFLIP        0xA1    // flip display horizontally
#define OLED_INVERT_OFF   0xA6    // set non-inverted display
#define OLED_INVERT       0xA7    // set inverse display
#define OLED_MULTIPLEX    0xA8    // set multiplex ratio (following byte)
#define OLED_DISPLAY_OFF  0xAE    // set display off (sleep mode)
#define OLED_DISPLAY_ON   0xAF    // set display on
#define OLED_PAGE         0xB0    // set start page (following byte)
#define OLED_YFLIP_OFF    0xC0    // don't flip display vertically
#define OLED_YFLIP        0xC8    // flip display vertically
#define OLED_OFFSET       0xD3    // set display offset (y-scroll: following byte)
#define OLED_VCOMH_LEVEL  0xD8    // set VCOMH
#define OLED_PRE_CHARGE   0xD9    // set pre-charge period
#define OLED_COMPINS      0xDA    // set COM pin config (following byte)


void ssd1306_initialise(void);
void ssd1306_verticalShift(uint8_t yPosition);
void ssd1306_drawBmp(uint8_t xPosition, uint8_t yPosition, uint8_t xSize, uint8_t ySize, const uint8_t* bmpData);
void ssd1306_printCharacter(char character);
void ssd1306_printString(char* string);
void ssd1306_printHexByte(uint8_t value);
void ssd1306_printHexWord(uint16_t value);
void ssd1306_setCursor(uint8_t xPosition, uint8_t yPosition);
void ssd1306_clearScreen(void);
void ssd1306_test(void);

#endif // __CH554_SSD1306_H__