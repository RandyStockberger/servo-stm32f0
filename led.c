// =============================================================================
//
#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "servo.h"
#include "led.h"
//
// Remap LED outputs to not interfer with SWD I/O
//
// ==============================================================================
// LED Defines
// ==============================================================================
//

#ifdef AVOID_SWD

led_t led[LED_COUNT] = {
	{ GPIOA, LED1A_PIN },
	{ GPIOA, LED1B_PIN },
	{ GPIOA, LED2A_PIN },
	{ GPIOB, LED2B_PIN },
	{ GPIOB, LED3A_PIN },
	{ GPIOA, LED3B_PIN },
	{ GPIOB, LED4A_PIN },
	{ GPIOB, LED4B_PIN },
};

#endif

#ifdef ORIGINAL

led_t led[LED_COUNT] = {
	{ GPIOA, LED1A_PIN },
	{ GPIOA, LED1B_PIN },
	{ GPIOA, LED2A_PIN },
	{ GPIOA, LED2B_PIN },
	{ GPIOA, LED3A_PIN },
	{ GPIOA, LED3B_PIN },
	{ GPIOB, LED4A_PIN },
	{ GPIOB, LED4B_PIN },
};

#endif	// ORIGINAL

#ifdef	VER22

led_t led[LED_COUNT] = {
	{ GPIOA, LED1A_PIN },
	{ GPIOA, LED1B_PIN },
	{ GPIOA, LED2A_PIN },
	{ GPIOA, LED2B_PIN },
	{ GPIOA, LED3A_PIN },
	{ GPIOA, LED3B_PIN },
	{ GPIOA, LED4A_PIN },
	{ GPIOA, LED4B_PIN },
};

#endif	// VER22
//
// ==============================================================================
// LED interface
// ==============================================================================
// 
// ledConfigPin -- Initialize a GPIO pin to drive an LED
//
// Initialize LED pins as output with no pullup or pulldown
//	Mode:Output, Type:Push-pull, Speed:low, PuPd:None
void ledConfigPin( enum eLed idx )
{
	uint32_t tmp;

	// Mode:Output (MODER:=1)
	tmp = led[idx].Port->MODER;
	tmp &= ~(3<<(led[idx].Pin*2));
	tmp |= (1<<led[idx].Pin*2);
	led[idx].Port->MODER = tmp;

	// Type:Push-pull (OTYPER:=0)
	led[idx].Port->OTYPER &= ~(1<<led[idx].Pin);

	// OSPEED:Low (OSPEEDR:=0)
	led[idx].Port->OSPEEDR &= ~(3<<(led[idx].Pin*2));

	//PUPD:None (PUPDR:=0)
	led[idx].Port->PUPDR &= (3<<(led[idx].Pin*2));
	
	// Make sure the LED is off
	led[idx].Port->BRR = 1<<led[idx].Pin;
}
//
// ==============================================================================
// ledInit -- Initialize all the LED GPIOs
//
void ledInit( void )
{
	for ( int idx=0; idx<LED_COUNT; ++idx ) {
		ledConfigPin( (enum eLed)idx );
	}
}
//
// ==============================================================================
// ledOff -- Set the pin high to turn the LED off
//
void ledOff( enum eLed idx )
{
	led[idx].Port->BRR = 1<<led[idx].Pin;
}
//
// ==============================================================================
// ledOn -- Set the pin low to turn the LED on
//
void ledOn( enum eLed idx )
{
	led[idx].Port->BSRR = 1<<led[idx].Pin;
}
//
// ==============================================================================
// ledToggle -- Toggle a led
//
void ledToggle( enum eLed idx )
{
	led[idx].Port->ODR ^= 1<<led[idx].Pin;
}
//
// ==============================================================================
