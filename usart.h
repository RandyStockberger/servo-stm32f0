// =============================================================================
//
#ifndef	__USART_H__

#ifdef USE_USART

// ==============================================================================
// USART - Simple UART interface without interrupts, DMA or buffering
// ==============================================================================
// STM32F072xB
// USART2
//	Function	Pin		AF
//	USART2_TX	2		1
//	USART2_RX	3		1
//
// STM32F030x6
// USART1
//	Function	Pin		AF
//	USART1_TX	2		1
//	USART1_RX	3		1
//
// ==============================================================================
// USART Defines
// ==============================================================================
//
// Uses USART1 on STM32F030x6 and USART2 on STM32F072xB
// TX:PA2:Output:AF1, RX:PA3:Input:AF1
//
// ==============================================================================
//
#ifdef STM32F072xB
#define DBG_USART			USART2
#endif	// STM32F072xB
//
#ifdef STM32F030x6
#define DBG_USART			USART1
#endif	// STM32F030x6
//
#define USART_GPIO			GPIOA
#define USART_TX_PIN		2
#define USART_RX_PIN		3
#define EOF		(-1)
//
char ulbuf[12];			// Buffer large enough to hold an unsigned long in decimal
//
// ==============================================================================
extern void usartInit( uint32_t baud );		// Initialize USART to baud rate, No parity, 8 bits, 1 stop bit
extern int usartReadByte( void );			// Read a byte from the USART, EOF in no data ready
extern int usartPutc( uint32_t ch );		// Write byte to USART, busy wait, returns char written
extern int usartTxEmpty();					// Return true if USART transmitter can take another character
extern int usartPuts( char const * str );	// Send a string to the UART
extern char *ultoa( unsigned long val, int base ); // Return ASCII representation of an unsigned long
extern void uprintf( char const *fmt, ... ); // Simple formatted output after printf()
extern void uputs( char * str );			// output a '\0' terminated string

#endif	//	__USART_H__
//
// ==============================================================================
