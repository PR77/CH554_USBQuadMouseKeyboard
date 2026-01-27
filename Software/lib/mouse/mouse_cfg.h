/********************************** (C) COPYRIGHT *******************************
* File Name     : MOUSE_CFG.H
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2026/01/27
* Description   : CH554 Mouse Buttons
*******************************************************************************/

#ifndef __CH554_MOUSE_CFG_H__
#define __CH554_MOUSE_CFG_H__

#include <stdint.h>
#include "ch554.h"

#ifndef LEFT_BUTTON_PORT
#define LEFT_BUTTON_PORT        0x90
#endif

#ifndef LEFT_BUTTON_MOD_OC
#define LEFT_BUTTON_MOD_OC      P1_MOD_OC
#endif

#ifndef LEFT_BUTTON_DIR_PU
#define LEFT_BUTTON_DIR_PU      P1_DIR_PU
#endif

#ifndef LEFT_BUTTON_PIN
#define LEFT_BUTTON_PIN         3
#endif

#ifndef RIGHT_BUTTON_PORT
#define RIGHT_BUTTON_PORT       0x90
#endif

#ifndef RIGHT_BUTTON_MOD_OC
#define RIGHT_BUTTON_MOD_OC     P1_MOD_OC
#endif

#ifndef RIGHT_BUTTON_DIR_PU
#define RIGHT_BUTTON_DIR_PU     P1_DIR_PU
#endif

#ifndef RIGHT_BUTTON_PIN
#define RIGHT_BUTTON_PIN        2
#endif

#endif // __CH554_MOUSE_CFG_H__