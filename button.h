// =============================================================================
//
#ifndef __BUTTON_H__

#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
//
#include "servo.h"
//
// ==============================================================================
// Button Defines
// ==============================================================================
//
// Number of buttons
#define BTN_COUNT		6
// Number of consecutive identical button reads before it is considered stable
#define BTN_DEBOUNCE	25
//
enum eBtn { BTN1, BTN2, BTN3, BTN4, BTNPLUS, BTNMINUS };

// Buttons are on GPIOA and GPIOB
#define BTN1_PIN		4
#define BTN2_PIN		5
#define BTN3_PIN		6
#define BTN4_PIN		7
#define BTNPLUS_PIN		0
#define BTNMINUS_PIN	1

#define BTN_GPIOA_MASK ((1<<BTN1_PIN)|(1<<BTN2_PIN)|(1<<BTN3_PIN)|(1<<BTN4_PIN))
#define BTN_GPIOB_MASK ((1<<BTNPLUS_PIN)|(1<<BTNMINUS_PIN))

typedef enum { BTN_DOWN, BTN_UP } btnstate_t;

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t			Pin;
	uint32_t		PinMask;
	btnstate_t		State;
	bool			Changed;
} btn_t;
//
// ==============================================================================
//
extern btn_t button[BTN_COUNT];
extern enum eBtn	btnLast;
extern uint32_t	btnBitsState;		// button map of previous debounced buttons
//uint32_t	btnChangedBits;		// map of which button(s) just changed
//
// ==============================================================================
// External button procedures
//
extern void btnInit( void );		// btnInit -- Initialize all the buttons
extern void btnISRProc( void );		// btnISRProc -- Poll the buttons on each heartbeat interrupt
//extern bool btnState( enum eBtn idx ); // btnState -- Get current state of the specified button

#endif	// __BUTTON_H__

//
// ==============================================================================
