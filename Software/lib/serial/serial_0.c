/********************************** (C) COPYRIGHT *******************************
* File Name     : SERIAL_0.c
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2025/02/06
* Description   : 8051 UART 0
*******************************************************************************/

#include <stdint.h>
#include <compiler.h>
#include "ch554.h"
#include "serial_0.h"

void serial_UART0Interrupt(void) __interrupt(INT_NO_UART0) {
	uint8_t receivedData = 0;

	if (RI) {
	    receivedData = SBUF;
		RI = 0;
	}

    if (TI) {
        TI = 0;
    }
}

void serial_initialiseSerial0(uint32_t baudRate, uint8_t alternativePins, uint8_t enableInterrupt) {
    SM0 = 0;
    SM1 = 1;
    SM2 = 0;                        // Serial port 0 usage mode 1
                                    // Use Timer1 as a baud rate generator
    RCLK = 0;                       // UART0 receive clock
    TCLK = 0;                       // UART0 transmit clock
    
    PCON = PCON | SMOD;
    TH1 = (0 - ((FREQ_SYS / 16L) / baudRate));
    TMOD = TMOD & ~bT1_GATE & ~bT1_CT & ~MASK_T1_MOD | bT1_M1;  // 0X20, Timer1 as 8-bit auto-reload timer
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;                         // Timer1 clock selection
    
    TR1 = 1;                        // Start timer 1
    REN = 1;                        // Serial 0 receive enable

    if (alternativePins) {
        PIN_FUNC = PIN_FUNC | bUART0_PIN_X;
    }

    if (enableInterrupt) {
        GPIO_IE = GPIO_IE | bIE_RXD0_LO;
    } else {
        GPIO_IE = GPIO_IE & ~bIE_RXD0_LO;
    }
}

inline void serial_disableSerial0Interrupt(void) {

    if (ES == 1) {
        ES = 0;                     // only if already enabled, then disable UART0 interrupt
    }
}

inline void serial_enableSerial0Interrupt(void) {

    if (ES == 0) {
        ES = 1;                     // only if already disabled, then emable UART0 interrupt
    }
}

uint8_t serial_getByteSerial0(void) {
    
    while (RI == 0);                // wait for receive interrupt flag (RI == 1)
    RI = 0;

    return (SBUF);
}

void serial_sendByteSerial0(uint8_t character) {
    SBUF = character;
    
    while (TI == 0);                // wait for transmit to finish (TI == 1)
    TI = 0;
}
