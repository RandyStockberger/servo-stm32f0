// =============================================================================
//
#ifndef __LED_H__

// ==============================================================================
// LED Defines
//
// ==============================================================================
//
#define LED_COUNT	8
//
enum eLed { LED1A, LED1B, LED2A, LED2B, LED3A, LED3B, LED4A, LED4B };

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t			Pin;
} led_t;

#ifdef AVOID_SWD
//
// LED mapping was changed to avoid use of the SWDIO/SWDCLK pins
//
#define LED1A_PIN	0
#define LED1B_PIN	1
#define LED2A_PIN	12
#define LED2B_PIN	3
#define LED3A_PIN	4
#define LED3B_PIN	15
#define LED4A_PIN	6
#define LED4B_PIN	7

#endif	// AVOID_SWD

#ifdef ORIGINAL

#define LED1A_PIN	0
#define LED1B_PIN	1
#define LED2A_PIN	12
#define LED2B_PIN	13
#define LED3A_PIN	14
#define LED3B_PIN	15
#define LED4A_PIN	6
#define LED4B_PIN	7

#endif	// ORIGINAL

#ifdef VER22

#define LED1A_PIN	0
#define LED1B_PIN	1
#define LED2A_PIN	4
#define LED2B_PIN	5
#define LED3A_PIN	6
#define LED3B_PIN	7
#define LED4A_PIN	12
#define LED4B_PIN	15

#endif	// VER22

led_t led[LED_COUNT];
//
// ==============================================================================
// LED interface
// ==============================================================================
//
extern void ledInit( void );				// ledInit -- Initialize all the LED GPIOs
extern void ledOff( enum eLed idx );		// ledOff -- Set the pin high to turn the LED off
extern void ledOn( enum eLed idx );			// ledOn -- Set the pin low to turn the LED on
extern void ledToggle( enum eLed idx );		// ledToggle -- Toggle a led

#endif	// __LED_H__
//
// ==============================================================================
