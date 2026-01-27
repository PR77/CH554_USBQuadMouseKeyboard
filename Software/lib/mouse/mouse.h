/********************************** (C) COPYRIGHT *******************************
* File Name     : MOUSE.H
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2025/11/09
* Description   : CH554 Amiga Mouse Emulation - middle layer SWC
*******************************************************************************/

#ifndef __CH554_MOUSE_H__
#define __CH554_MOUSE_H__

#include <stdint.h>
#include "ch554.h"

#define DPI_MOVEMENT_FILTER 1

/**
 * Mouse button masks - used for the first byte in the HID report.
 */
#define MOUSE_BUTTON_LEFT   0x01
#define MOUSE_BUTTON_MIDDLE 0x04
#define MOUSE_BUTTON_RIGHT  0x02

typedef struct {
    uint8_t buttonState;
    int8_t xAxisMovement;
    int8_t yAxisMovement;
    int8_t wheelMovement;
} devTypeMousePayload_s;

typedef enum {
    dontInvertButtons = 0,
    invertButtons
} buttonsInvert_e;

void mouse_initialise(buttonsInvert_e invertState);
void mouse_deinitialise(void);
void mouse_leftButton(uint8_t buttonState);
void mouse_rightButton(uint8_t buttonState);
uint8_t mouse_translateMovement(devTypeMousePayload_s *rawMouseReport);
char * mouse_getButtonString(devTypeMousePayload_s *rawMouseReport);

#endif // __CH554_MOUSE_H__