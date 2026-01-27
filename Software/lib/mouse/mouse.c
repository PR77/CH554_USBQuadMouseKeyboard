/********************************** (C) COPYRIGHT *******************************
* File Name     : MOUSE.C
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2025/11/09
* Description   : CH554 Amiga Mouse Emulation - middle layer SWC
*******************************************************************************/

#include <stdint.h>
#include <compiler.h>
#include <string.h>
#include <stdlib.h>
#include "ch554.h"
#include "quadrature.h"
#include "mouse.h"
#include "mouse_cfg.h"

SBIT(LEFT_BUTTON, LEFT_BUTTON_PORT, LEFT_BUTTON_PIN);
SBIT(RIGHT_BUTTON, RIGHT_BUTTON_PORT, RIGHT_BUTTON_PIN);

static __xdata devTypeMousePayload_s previousRawMouseReport;
static __xdata char buttonString[4];
static __xdata buttonsInvert_e invertButtonLogic = dontInvertButtons;

void mouse_initialise(buttonsInvert_e invertState) {

    LEFT_BUTTON_MOD_OC = LEFT_BUTTON_MOD_OC & ~(1 << LEFT_BUTTON_PIN);
    LEFT_BUTTON_DIR_PU = LEFT_BUTTON_DIR_PU | (1 << LEFT_BUTTON_PIN);

    RIGHT_BUTTON_MOD_OC = RIGHT_BUTTON_MOD_OC & ~(1 << RIGHT_BUTTON_PIN);
    RIGHT_BUTTON_DIR_PU = RIGHT_BUTTON_DIR_PU | (1 << RIGHT_BUTTON_PIN);

    invertButtonLogic = invertState;

    if (invertButtonLogic == invertButtons) {
        LEFT_BUTTON = 1;
        RIGHT_BUTTON = 1;
    } else {
        LEFT_BUTTON = 0;
        RIGHT_BUTTON = 0;
    }

    quadrature_initialise(encodingRate8000Hz);

    memset(&previousRawMouseReport, 0, sizeof(devTypeMousePayload_s));
    strncpy(buttonString, "---", sizeof(buttonString));
}

void mouse_deinitialise(void) {
    quadrature_deinitialise();
}

void mouse_leftButton(uint8_t buttonState) {

    if (buttonState ^ (uint8_t)invertButtonLogic) {
        LEFT_BUTTON = 1;
    } else {
        LEFT_BUTTON = 0;
    }
}

void mouse_rightButton(uint8_t buttonState) {

    if (buttonState ^ (uint8_t)invertButtonLogic) {
        RIGHT_BUTTON = 1;
    } else {
        RIGHT_BUTTON = 0;
    }
}

uint8_t mouse_translateMovement(devTypeMousePayload_s *rawMouseReport) {

    int8_t movementAmount = 0;
    uint8_t movementUpdate = 0;
    
    // NULL pointer check - this function is called with the address of RxBuffer
    // and need to ensure this buffer is not at 0.
    if (NULL == rawMouseReport) {
        return (0);
    }

    if (rawMouseReport->xAxisMovement != previousRawMouseReport.xAxisMovement) {
        previousRawMouseReport.xAxisMovement = rawMouseReport->xAxisMovement;
        movementAmount = (rawMouseReport->xAxisMovement) / DPI_MOVEMENT_FILTER;
        quadrature_updateCounts(QUADRATURE_X_CHANNEL, movementAmount);
        movementUpdate = 1;
    }

    if (rawMouseReport->yAxisMovement != previousRawMouseReport.yAxisMovement) {
        previousRawMouseReport.yAxisMovement = rawMouseReport->yAxisMovement;
        movementAmount = (rawMouseReport->yAxisMovement) / DPI_MOVEMENT_FILTER;
        quadrature_updateCounts(QUADRATURE_Y_CHANNEL, movementAmount);
        movementUpdate = 1;
    }

    if (rawMouseReport->buttonState & MOUSE_BUTTON_LEFT) {
        mouse_leftButton(1);

    } else {
        mouse_leftButton(0);
    }

    if (rawMouseReport->buttonState & MOUSE_BUTTON_MIDDLE) {
        // ...
    } else {
        // ...
    }    

    if (rawMouseReport->buttonState & MOUSE_BUTTON_RIGHT) {
        mouse_rightButton(1);
    } else {
        mouse_rightButton(0);
    }    

    return (movementUpdate);
}

char * mouse_getButtonString(devTypeMousePayload_s *rawMouseReport) {

    // Button string is made up of 3 characters (plus terminator). For example,
    // with no buttons pressed "---", left "L--", middle "-M-" and right "--R".
    strncpy(buttonString, "---", sizeof(buttonString));

    // NULL pointer check
    if (NULL != rawMouseReport) {
        if (rawMouseReport->buttonState & MOUSE_BUTTON_LEFT) {
            buttonString[0] = 'L';
        }
        
        if (rawMouseReport->buttonState & MOUSE_BUTTON_MIDDLE) {
            buttonString[1] = 'M';
        }
        
        if (rawMouseReport->buttonState & MOUSE_BUTTON_RIGHT) {
            buttonString[2] = 'R';
        }
    }
    
    return (buttonString);
}

