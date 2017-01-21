// =============================================================================
//
#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "button.h"
#include "led.h"
#include "pwm.h"
//
// Initialize button data structures
btn_t button[BTN_COUNT] = {
	// GPIO		Pin Number		Pin Mask			Button State	Changed?
	{ GPIOA,	BTN1_PIN,		1<<BTN1_PIN,		BTN_UP,			true },	// BTN1
	{ GPIOA,	BTN2_PIN,		1<<BTN2_PIN, 		BTN_UP,			true },	// BTN2
	{ GPIOA,	BTN3_PIN,		1<<BTN3_PIN,		BTN_UP,			true },	// BTN3
	{ GPIOA,	BTN4_PIN,		1<<BTN4_PIN,		BTN_UP,			true },	// BTN4
	{ GPIOB,	BTNPLUS_PIN,	1<<BTNPLUS_PIN,		BTN_UP,			true },	// BTNPLUS
	{ GPIOB,	BTNMINUS_PIN,	1<<BTNMINUS_PIN,	BTN_UP,			true },	// BTNMINUS
};
//
// ==============================================================================
//
uint32_t	btnBitsState;		// button map of previous debounced buttons
//
// ==============================================================================
// Button interface
// ==============================================================================
// btnConfigPin -- Configure the GPIO pin associated with a button
// Configure the pin as an input with pullup enabled
//
static void btnConfigPin( enum eBtn idx )
{
	// Buttons are input, low speed, push/pull with pullup enabled
	// Input (MODER:=0)
	button[idx].Port->MODER &= ~(3<<(button[idx].Pin*2));

	// Output is push/pull (OTYPER:=0)
	button[idx].Port->OTYPER &= ~(1<<button[idx].Pin);

	// Speed is low (OSPEEDR:=0)
	button[idx].Port->OSPEEDR &= ~(3<<(button[idx].Pin*2));

	// Pullup enabled (PUPDR:=1)
	button[idx].Port->PUPDR &= ~(3<<(button[idx].Pin*2));
	button[idx].Port->PUPDR |=  (1<<(button[idx].Pin*2));
}
//
// ==============================================================================
// configButton -- Initialize all the buttons
//
void btnInit( void )
{
	for ( int idx=0; idx<BTN_COUNT; ++idx ) {
		btnConfigPin( (enum eBtn) idx );
	}
}
//
// ==============================================================================
// Button procs...
//
// Each of these procedures is called when the associated state change is detected
static void btn1Down( void )
{
	ledOff( LED1B );
	ledOn( LED1A );
	servo[(int)SERVO1].targetPos = servo[(int)SERVO1].maxPos;
}

static void btn1Up( void )
{
	ledOff( LED1A );
	ledOn( LED1B );
	servo[(int)SERVO1].targetPos = servo[(int)SERVO1].minPos;
}

static void btn2Down( void )
{
	ledOff( LED2B );
	ledOn( LED2A );
	servo[(int)SERVO2].targetPos = servo[(int)SERVO2].maxPos;
}

static void btn2Up( void )
{
	ledOff( LED2A );
	ledOn( LED2B );
	servo[(int)SERVO2].targetPos = servo[(int)SERVO2].minPos;
}

static void btn3Down( void )
{
	ledOff( LED3B );
	ledOn( LED3A );
	servo[(int)SERVO3].targetPos = servo[(int)SERVO3].maxPos;
}

static void btn3Up( void )
{
	ledOff( LED3A );
	ledOn( LED3B );
	servo[(int)SERVO3].targetPos = servo[(int)SERVO3].minPos;
}

static void btn4Down( void )
{
	ledOff( LED4B );
	ledOn( LED4A );
	servo[(int)SERVO4].targetPos = servo[(int)SERVO4].maxPos;
}

static void btn4Up( void )
{
	ledOff( LED4A );
	ledOn( LED4B );
	servo[(int)SERVO4].targetPos = servo[(int)SERVO4].minPos;
}

static void btnPlusDown( void )
{
//	ledOff( LED1B );
//	ledOn( LED1A );
//	servo[(int)SERVO1].targetPos = servo[(int)SERVO1].maxPos;
}

static void btnPlusUp( void )
{
//	ledOff( LED1A );
//	ledOn( LED1B );
//	servo[(int)SERVO1].targetPos = servo[(int)SERVO1].minPos;
}

static void btnMinusDown( void )
{
	//ledToggle( LED1B );
}

static void btnMinusUp( void )
{
//	ledToggle( LED1B );
}

//
// ==============================================================================
// btnISRProc -- Poll the buttons on each heartbeat interrupt to determine which have changed
//
// This routine is called once for each heartbeat interrupt (1000Hz)
//
// btnBitsState   - Previous state of the buttons
// btnBitsLast    - Current state of the buttons, not yet stable/debounced
// btnInputCount  - How many heartbeats btnBitsLast has been stable
// btnBits        - State of the buttons as read on current iteration
// btnChangedBits - Bitmap of buttons that changed between current and previous btnBitsState
//
void btnISRProc( void )
{
	uint32_t btnBits;					// button map as just read
	static uint32_t btnBitsLast = 0;	// button map from last iteration
	static uint32_t	btnInputCount;		// Count how many times this bit pattern has been seen

	// New code using a jump table
	static void (* const btnProc[])(void) = {
		btn1Down,		btn1Up,
		btn2Down,		btn2Up,
		btn3Down,		btn3Up,
		btn4Down,		btn4Up,
		btnPlusDown,	btnPlusUp,
		btnMinusDown,	btnMinusUp,
	};

	btnBits = GPIOA->IDR & BTN_GPIOA_MASK;
	btnBits |= GPIOB->IDR & BTN_GPIOB_MASK;

	++btnInputCount;
	if ( btnBits != btnBitsLast ) {		// Something changed since last iteration
		btnBitsLast = btnBits;			// Save new state
		btnInputCount = 0;				// Reset count
	}

	if ( (btnBitsState != btnBitsLast) && (BTN_DEBOUNCE == btnInputCount) ) {
		for ( int idx=0; idx<BTN_COUNT; ++idx ) {
			if ( (btnBitsState & button[idx].PinMask) != (btnBitsLast & button[idx].PinMask) ) {
				// button has changed state
				if ( btnBitsLast & button[idx].PinMask ) {
					btnProc[(idx<<1)+1]();	// Button down proc
				}
				else {
					btnProc[idx<<1]();	// Button up proc
				}
			}
		}
		btnBitsState = btnBitsLast;
	}
}
//
// ==============================================================================
