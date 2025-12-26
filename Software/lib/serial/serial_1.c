/********************************** (C) COPYRIGHT *******************************
* File Name     : SERIAL_1.C
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2024/12/17
* Description   : 8051 UART 1
*******************************************************************************/

#include <stdint.h>
#include <compiler.h>
#include "ch554.h"
#include "tick.h"
#include "system.h"
#include "serial_1.h"

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
static volatile __xdata uint8_t serial_transmitBuffer[SERIAL_1_TX_BUFFER_SIZE];
static volatile __xdata uint8_t serial_transmitWriteIndex;
static volatile __xdata uint8_t serial_transmitReadIndex;
static volatile __xdata uint8_t serial_transmitTriggerred;
#endif

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
static volatile __xdata uint8_t serial_receiveBuffer[SERIAL_1_RX_BUFFER_SIZE];
static volatile __xdata uint8_t serial_receiveWriteIndex;
static volatile __xdata uint8_t serial_receiveReadIndex;
#endif

void serial_UART1Interrupt(void) __interrupt(INT_NO_UART1) {

	if (U1RI) {
#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
        volatile uint8_t nextWriteIndex = 0, receivedData = 0;
#endif

		U1RI = 0;
#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
        receivedData = SBUF1;

        nextWriteIndex = (serial_receiveWriteIndex + 1) & SERIAL_1_RX_BUFFER_MASK;
    
        if (nextWriteIndex != serial_receiveReadIndex) {
            // Check if receive buffer is not full, otherwise need to drop
            // received data.

            serial_receiveBuffer[serial_receiveWriteIndex] = receivedData;
            serial_receiveWriteIndex = nextWriteIndex;
        }
#endif
	}

    if (U1TI) {
        U1TI = 0;

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
        // Check if transmit buffer is not empty
        if (serial_transmitWriteIndex != serial_transmitReadIndex) {

            // Get one byte from buffer and write it
            SBUF1 = serial_transmitBuffer[serial_transmitReadIndex];

            // Calculate and store new buffer index
            serial_transmitReadIndex = (serial_transmitReadIndex + 1) & SERIAL_1_TX_BUFFER_MASK;
        } else {
            // If there are no more bytes to be sent, then clear the Transmit Triggered
            // flag to ensure a transmit is (re-)triggered when another byte(s) is added
            // to the transmit buffer.
            serial_transmitTriggerred = 0;
        }
#endif
    }
}

void serial_initialiseSerial1(uint32_t baudRate, uint8_t alternativePins) {
    U1SM0 = 0;
    U1SMOD = 1;
    U1REN = 1;

    SBAUD1 = (0 - ((FREQ_SYS / 16L) / baudRate));

    if (alternativePins) {
        PIN_FUNC = PIN_FUNC | bUART1_PIN_X;
    }

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
    serial_transmitWriteIndex = 0;
    serial_transmitReadIndex = 0;
    serial_transmitTriggerred = 0;
#endif

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
    serial_receiveWriteIndex = 0;
    serial_receiveReadIndex = 0;
#endif

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
    GPIO_IE = GPIO_IE | bIE_RXD1_LO;
#else
    GPIO_IE = GPIO_IE & ~bIE_RXD1_LO;
#endif

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS) || defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
    U1TI = 0;
    U1RI = 0;
    serial_enableSerial1Interrupt();
#endif
}

inline void serial_disableSerial1Interrupt(void) {

    if (IE_UART1 == 1) {
        IE_UART1 = 0;                    // only if already enabled, then disable UART1 interrupt
    }
}

inline void serial_enableSerial1Interrupt(void) {

    if (IE_UART1 == 0) {
        IE_UART1 = 1;                    // only if already disabled, then emable UART1 interrupt
    }
}

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
uint16_t serial_isDataAvailableSerial1Interrupt(void) {

    // If receive buffer is not empty, then return RECEIVE_DATA_AVAIL.
    if (serial_receiveWriteIndex != serial_receiveReadIndex) {
        return (RECEIVE_DATA_AVAIL);
    }

    return (RECEIVE_NO_DATA_AVAIL);
}

uint16_t serial_getByteSerial1Interrupt(uint32_t timeout) {
    uint16_t receivedData = RECEIVE_NO_DATA_AVAIL;
    uint32_t previousCountTimeout = tick_get1msTimerCount();

    // If receive buffer is empty, then block for timeout period only if
    // timeout period is !0.
    // If timeout occurs, then break out or report RECEIVE_TIMEOUT.
    if (timeout != 0) {
        while (serial_receiveWriteIndex == serial_receiveReadIndex) {

            if ((tick_get1msTimerCount() - previousCountTimeout) > timeout) {
                return (RECEIVE_TIMEOUT);
            }
        }
    }

    serial_disableSerial1Interrupt();

    // If receive buffer is not empty
    if (serial_receiveWriteIndex != serial_receiveReadIndex) {
        // Get one byte from buffer and pass it along to the caller
        receivedData = serial_receiveBuffer[serial_receiveReadIndex];

        // Calculate and store new buffer index
        serial_receiveReadIndex = (serial_receiveReadIndex + 1) & SERIAL_1_RX_BUFFER_MASK;
    }

    serial_enableSerial1Interrupt();
    
    return (receivedData);
}
#endif // SERIAL_1_ENABLE_RX_INTERRUPTS

#if !defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
uint16_t serial_getByteSerial1Blocking(uint32_t timeout) {
    uint32_t previousCountTimeout = tick_get1msTimerCount();

    // If receive indicator is not set, then block for timeout period
    // otherwise break out and report RECEIVE_TIMEOUT.
    while (U1RI == 0) {

        if ((tick_get1msTimerCount() - previousCountTimeout) > timeout) {
            return (RECEIVE_TIMEOUT);
        }
    }

    U1RI = 0;

    return (SBUF1);
}
#endif // ! SERIAL_1_ENABLE_RX_INTERRUPTS

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
void serial_sendByteSerial1Interrupt(uint8_t character) {

    volatile uint8_t nextWriteIndex = (serial_transmitWriteIndex + 1) & SERIAL_1_TX_BUFFER_MASK;;

    while (nextWriteIndex == serial_transmitReadIndex) {
        // Wait for free space in buffer before pushing next character into it.
        // When this occurs, then sendByte...() will be blocking.
        ;
    }

    serial_transmitBuffer[serial_transmitWriteIndex] = character;
    serial_transmitWriteIndex = nextWriteIndex;

    if ((U1TI == 0) && (serial_transmitTriggerred == 0)) {
        // Trigger transmission if UART is not busy and has not already been triggered.
        serial_transmitTriggerred = 1;
        U1TI = 1;
    }
}
#endif // SERIAL_1_ENABLE_TX_INTERRUPTS

#if !defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
void serial_sendByteSerial1Blocking(uint8_t character) {
    SBUF1 = character;
    
    while (U1TI == 0) {
        // Wait for transmit to finish (TI == 1)
        ;
    }

    U1TI = 0;
}
#endif // ! SERIAL_1_ENABLE_TX_INTERRUPTS
