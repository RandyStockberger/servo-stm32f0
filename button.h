// =============================================================================
//
#ifndef __BUTTON_H__

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
//
enum eBtnState { BTN1DOWN, BTN1UP, BTN2DOWN, BTN2UP, BTN3DOWN, BTN3UP, BTN4DOWN, BTN4UP, UNDEF };
//
// Buttons are on GPIOA and/or GPIOB

#ifdef ORIGINAL

#define BTN1_PIN		4
#define BTN2_PIN		5
#define BTN3_PIN		6
#define BTN4_PIN		7
#define BTNPLUS_PIN		0
#define BTNMINUS_PIN	1

#define BTN_GPIOA_MASK ((1<<BTN1_PIN)|(1<<BTN2_PIN)|(1<<BTN3_PIN)|(1<<BTN4_PIN))
#define BTN_GPIOB_MASK ((1<<BTNPLUS_PIN)|(1<<BTNMINUS_PIN))

#endif	// ORIGINAL

#ifdef VER22

#define BTNPLUS_PIN		0
#define BTNMINUS_PIN	1
#define BTN1_PIN		3
#define BTN2_PIN		4
#define BTN3_PIN		6
#define BTN4_PIN		7

#define BTN_GPIOB_MASK ( (1<<BTNPLUS_PIN) | (1<<BTNMINUS_PIN) | (1<<BTN1_PIN) | (1<<BTN2_PIN) | (1<<BTN3_PIN) | (1<<BTN4_PIN) )

#endif // VER22

typedef enum { BTN_DOWN, BTN_UP } btnstate_t;

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t			Pin;
	uint32_t		PinMask;
} btn_t;
//
// ==============================================================================
//
extern btn_t button[BTN_COUNT];
extern uint32_t	btnBitsState;		// button map of previous debounced buttons
//
// ==============================================================================
// External button procedures
//
extern void btnInit( void );		// btnInit -- Initialize all the buttons
extern void btnISRProc( void );		// btnISRProc -- Poll the buttons on each heartbeat interrupt
extern int btnIsDown( enum eBtn );	// btnIsDown -- Return current button/switch state

#endif	// __BUTTON_H__

//
// ==============================================================================
