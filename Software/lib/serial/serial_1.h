/********************************** (C) COPYRIGHT *******************************
* File Name     : SERIAL.H
* Author        : Paul Raspa (PR77)
* License       : MIT
* Version       : V1.0
* Date          : 2024/12/17
* Description   : 8051 UART 1
*******************************************************************************/

#ifndef __CH554_SERIAL_1_H__
#define __CH554_SERIAL_1_H__

#include <stdint.h>
#include "ch554.h"
#include "serial_cfg.h"

#define SERIAL_1_ENABLE_TX_INTERRUPTS
#define SERIAL_1_ENABLE_RX_INTERRUPTS

#ifndef SERIAL_1_TX_BUFFER_SIZE
#define SERIAL_1_TX_BUFFER_SIZE   64
#endif

#ifndef SERIAL_1_RX_BUFFER_SIZE
#define SERIAL_1_RX_BUFFER_SIZE   64
#endif

#define SERIAL_1_TX_BUFFER_MASK   (SERIAL_1_TX_BUFFER_SIZE - 1)
#define SERIAL_1_RX_BUFFER_MASK   (SERIAL_1_RX_BUFFER_SIZE - 1)

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS) && (SERIAL_1_TX_BUFFER_SIZE & SERIAL_1_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS) && (SERIAL_1_RX_BUFFER_SIZE & SERIAL_1_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2   
#endif  

void serial_UART1Interrupt(void) __interrupt(INT_NO_UART1);
void serial_initialiseSerial1(uint32_t baudRate, uint8_t alternativePins);
inline void serial_disableSerial1Interrupt(void);
inline void serial_enableSerial1Interrupt(void);

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
void serial_sendByteSerial1Interrupt(uint8_t character);
#else
void serial_sendByteSerial1Blocking(uint8_t character);
#endif // SERIAL_1_ENABLE_TX_INTERRUPTS

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
uint16_t serial_isDataAvailableSerial1Interrupt(void);
uint16_t serial_getByteSerial1Interrupt(uint32_t timeout);
#else
uint16_t serial_getByteSerial1Blocking(uint32_t timeout);
#endif // SERIAL_1_ENABLE_RX_INTERRUPTS

#if defined(SERIAL_1_ENABLE_TX_INTERRUPTS)
#define serial_sendByteSerial1(character)   serial_sendByteSerial1Interrupt(character)
#else
#define serial_sendByteSerial1(character)   serial_sendByteSerial1Blocking(character)
#endif // SERIAL_1_ENABLE_TX_INTERRUPTS

#if defined(SERIAL_1_ENABLE_RX_INTERRUPTS)
#define serial_isDataAvailable()            serial_isDataAvailableSerial1Interrupt()
#define serial_getByteSerial1(timeout)      serial_getByteSerial1Interrupt(timeout)
#else
#define serial_getByteSerial1(timeout)      serial_getByteSerial1Blocking(timeout)
#endif // SERIAL_1_ENABLE_RX_INTERRUPTS

#endif // __CH554_SERIAL_1_H__