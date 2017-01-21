// =============================================================================
//
#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "usart.h"
//
// =============================================================================
//

#ifdef USE_USART

// ==============================================================================
// USART Defines
//
// USART1 on STM32F030x6, USART2 on STM32F072xB
// TX:PA2:Output:AF1, RX:PA3:Input:AF1
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
#endif	// USE_USART
//
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
//
static const char uldigits[] = "0123456789ABCDEF";
char ulbuf[12];			// Buffer large enough to hold an unsigned long in decimal
//
// ==============================================================================
// usartInit -- Initialize USART to baud rate, no parity, 8 data bits, 1 stop bit
void usartInit( uint32_t baud )
{
	uint32_t tmp;

	// USART_TX_PIN:PA2:Output, USART_RX_PIN:PA3:Input
#ifdef STM32F072xB
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;		// USART2 clock on
#endif
#ifdef STM32F030x6
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		// USART1 clock on
#endif

	// Configure PA3(USART_RX_PIN) and PA2(USART_TX_PIN) as Alternate Function
	tmp = USART_GPIO->MODER & ~( 0x03<<(USART_RX_PIN*2) | 0x03<<(USART_TX_PIN*2) );
	USART_GPIO->MODER = tmp | ((2<<(USART_RX_PIN*2)) | 2<<(USART_TX_PIN*2));

	// Configure as OTYPER 0==push-pull, 1==open-drain
	USART_GPIO->OTYPER &= ~((1<<USART_RX_PIN) | (1<<USART_TX_PIN));
	USART_GPIO->OTYPER |= ((0<<USART_RX_PIN) | (0<<USART_TX_PIN));

	// 0bx0==Low Speed, 0b01==Medium speed, 0b11==High speed
	USART_GPIO->OSPEEDR |= ( (3<<(USART_RX_PIN*2)) | (3<<(USART_TX_PIN*2)) );

	// 0b00==No PU/PD, 0b01==Pull up, 0b10==Pull down
	USART_GPIO->PUPDR &= ~( (0x03<<(USART_RX_PIN*2)) | (0x03<<(USART_TX_PIN*2)) );
//rls	USART_GPIO->PUPDR |= ( (1<<(USART_RX_PIN*2)) | (1<<(USART_TX_PIN*2)) );

	// Connect USART_TX_PIN:PA2 and PA3 to DBG_USART
	USART_GPIO->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL2);
	USART_GPIO->AFR[0] |= ((1 << (3*4)) | (1 << (2*4)));

	DBG_USART->CR1 = ( USART_CR1_TE | USART_CR1_RE );
	//DBG_USART->BRR = (SystemCoreClock/2) / baud;
	DBG_USART->BRR = (SystemCoreClock) / baud;
	DBG_USART->CR1 |= USART_CR1_UE;
}
//
// ==============================================================================
// usartReadByte -- Read a byte from the USART, EOF in no data ready
int usartReadByte( void )
{
	if ( DBG_USART->ISR & USART_ISR_RXNE )
		return (int)DBG_USART->RDR;
	return EOF;
}
//
// ==============================================================================
// usartPutc -- Write byte to USART, busy wait, returns char written
int usartPutc( uint32_t ch )
{
#if 0
	if ( '\n' == ch ) {
		usartPutc( '\r' );
	}
#endif

	while( ! (DBG_USART->ISR & USART_ISR_TXE) )
		;
	DBG_USART->TDR = ch;

	return ch;
}
//
// ==============================================================================
// usartTxEmpty -- Return true if USART transmitter can take another character
int usartTxEmpty()
{
	return (DBG_USART->ISR & USART_ISR_TXE) ? 1 : 0; 
}
//
// ==============================================================================
// usartPuts -- Send a string to the UART
//
// Return:	Non-negative value on success
//			EOF on failure
//
int usartPuts( char const * str )
{
	while ( *str ) {
		usartPutc( *str );
		++str;
	}

	return 0;
}
//
// ==============================================================================
// ultoa -- Return ASCII representation of an unsigned long
//
// Return:	Pointer to a static buffer containing the resultant string
//
char *ultoa( unsigned long val, int base )
{
	char * ptr = &ulbuf[11];		// Point to end of buffer

	*ptr = '\0';

	do {
		--ptr;
		*ptr = uldigits[val % base];
		val /= base;
	} while ( val > 0 );

	return ptr;
}
//
// ==============================================================================
//
// uprintf -- Simple formatted output after printf()
//
// Return:	Number of characters sent to output
//
// Supports format specifiers '%', 'c', 's', 'd', 'x'
void uprintf( char const *fmt, ... )
{
	va_list arg;
//	int len = 0;
	int ch;
	int val, base;
	unsigned int uval;
	char * str;
	//char buf[10];
usartPutc('1');
	va_start( arg, fmt );
usartPutc('2');
	while( (ch = *fmt++) ) {
usartPutc('3');
		if ( '%' == ch ) {
			switch( ch = *fmt++ ) {
			case '%':			// "%%" outputs a single "%"
				usartPutc( ch );
				break;

			case 'c':
				usartPutc( va_arg( arg, int ) );
				break;

			case 's':
				str = va_arg( arg, char * );
				usartPuts( str );
				break;

			case 'd':
				val = va_arg( arg, int );
				if ( val < 0 ) {
					usartPutc( '-' );
					val = - val;
				}
				str = ultoa( (unsigned long) val, 10 );
				usartPuts( str );
				break;

			case 'x':
			case 'u':
				base = ( ch == 'u' ) ? 10 : 16;
				uval = va_arg( arg, unsigned int );
				str = ultoa( (unsigned long)uval, base );
				usartPuts( str );
				break;
			}
		}
		else {
			usartPutc( ch );
		}
	}
	va_end( arg );
}
//
// ==============================================================================
//
void uputs( char * str )
{
	while ( *str ) {
		usartPutc( *str++ );
	}

}
//
#endif	// USE_USART
//
// ==============================================================================
