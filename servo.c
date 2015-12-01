// ============================================================================
//
// // 												  (Nucleo)
// Pin Name		Function		STM32F030xx			STM32F072RB
// 
// VDD			SYSTEM				 1						 CN7:5
// PF0-OSC_IN						 2				5		 CN7:29
// PF1-OSC_OUT						 3				6		 CN7:31
// NRST			SYSTEM				 4				7		 CN7:14
// VDDA			SYSTEM				 5				13		CN10:7
// PA0			LED1A				 6				14		 CN7:28
// PA1			LED1B				 7				15		 CN7:30
// PA2			TX					 8				16		CN10:35
// PA3			RX					 9				17		CN10:37
// PA4			BTN1				10				20		 CN7:32
// PA5			BTN2/Nucleo LED		11				21		CN10:11
// PA6			BTN3				12				22		CN10:13
// PA7			BTN4				13				23		CN10:15
// PB0			BTNPLUS				14				26		 CN7:34
// PB1			BTNMINUS			15				27		CN10:24
// VSS			SYSTEM				16				31		 CN7:8
// VDD			SYSTEM				17				32		 CN7:15
// PA8			SERVO1				18				41		CN10:23
// PA9			SERVO2				19				42		CN10:21
// PA10			SERVO3				20				43		CN10:33
// PA11			SERVO4				21				44		CN10:14
// PA12			LED2A				22				45		CN10:12
// PA13			SWDIO				23				46		 CN7:13
// PA14			SWCLK				24				49		 CN7:15
// PA15			LED2B				25				50		 CN7:17
// PB3			LED3A				26				55		CN10:31
// PB4			LED3B				27				56		CN10:27
// PB5			DCCIN				28				57		CN10:29
// PB6			LED4A				29				58		CN10:17
// PB7			LED4B				30				59		 CN7:21
// BOOT0		SYSTEM				31				60		 CN7:7
// VSS			SYSTEM				32				63		CN10:20
// 
// PWM outputs:
//	TIM1_CH1:PA8
//	TIM1_CH2:PA9
//	TIM1_CH3:PA10
//	TIM1_CH4:PA11
//	TIM3_CH1:PA6,PB4
//	TIM3_CH2:PA7,PB5
//	TIM3_CH3:PB0
//	TIM3_CH4:PB1
//	TIM14_CH1:PA4,PA7,PB1
//	TIM15_CH1:PA2
//	TIM15_CH2:PA3
//	TIM16_CH1:PA6
//	TIM17_CH1:PA7

// Using Nucleo-F072RB board for prototyping or using SERVOBoard with STM32F030K6T6
#define NUCLEO
// USART can be used for debugging
#define USE_USART

#include <stdint.h>
#include <stdbool.h>
//
#include "stm32f0xx.h"
#include "system_stm32f0xx.h"
//
//#define HB_HZ	1000
#define HB_HZ	50

#ifdef USE_USART

#define EOF		(-1)

#endif	// USE_USART

// ----------------------------------------------------------------------------
// Servo Defines

// There are 4 servos
#define SERVO_COUNT		4

#define SERVO1			0
#define SERVO2			1
#define SERVO3			2
#define SERVO4			3

// Servo pins
#define	SERVO1_PIN		 8
#define	SERVO2_PIN		 9
#define	SERVO3_PIN		10
#define	SERVO4_PIN		11

// Amount to move each servo on each timer tick
#define SERVO_DELTA	3
//
// Maximum and minimum PWM values
// PWM_MIN - electrify standard absolute minimum is approx 200
#define PWM_MIN			1150	// Clockwise when looking at shaft
// PWM_MAX - electrify standard absolute maximum is approx 2280
#define PWM_MAX			1700
// Initialize to mid point
#define PWM_MID			((PWM_MIN+PWM_MAX)/2)
//
#define PWM_TIMER_HZ	1000000UL
// Prescale PWM timer to run at PWM_TIMER_HZ
#define PWM_PRESCALE	(48-1)
// PWM timer period is 20mS (20,000uS)
#define PWM_MAX_COUNT	(PWM_TIMER_HZ)/(HB_HZ)
//
// PWM speed -- number of seconds to go from end to end
#define PWM_SPEED		5
// PWM pulses per second on each servo
#define PWM_HZ			(PWM_TIMER_HZ)/(PWM_MAX_COUNT)
// How much to increment(decrement) servo position for each servo pulse
#define PWM_STEPSIZE	(PWM_MAX-PWM_MIN)/(PWM_SPEED*PWM_HZ)

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t			Pin;
	uint16_t		currentPos;
	uint16_t		targetPos;
} servo_t;

servo_t servo[SERVO_COUNT] = {
	{ GPIOA,	SERVO1_PIN, (PWM_MID), (PWM_MID) },
	{ GPIOA,	SERVO2_PIN, (PWM_MID), (PWM_MID) },
	{ GPIOA,	SERVO3_PIN, (PWM_MID), (PWM_MID) },
	{ GPIOA,	SERVO4_PIN, (PWM_MID), (PWM_MID) },
};

// ----------------------------------------------------------------------------
// Button Defines
//
// Number of buttons
#define BTN_COUNT		6

#define BTN1			0
#define BTN2			1
#define BTN3			2
#define BTN4			3
#define BTNPLUS			4
#define BTNMINUS		5

// Buttons are on GPIOA and GPIOB
#define BTN1_PIN		4
#define BTN2_PIN		5
#define BTN3_PIN		6
#define BTN4_PIN		7
#define BTNPLUS_PIN		0
#define BTNMINUS_PIN	1

typedef enum { BTN_DOWN, BTN_UP } btnstate_t;

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t		Pin;
	btnstate_t	State;
	bool		Changed;
} btn_t;

btn_t button[BTN_COUNT] = {
	{ GPIOA, BTN1_PIN,		BTN_UP, true },	// BTN1
	{ GPIOA, BTN2_PIN,		BTN_UP, true },	// BTN2
	{ GPIOA, BTN3_PIN,		BTN_UP, true },	// BTN3
	{ GPIOA, BTN4_PIN,		BTN_UP, true },	// BTN4
	{ GPIOB, BTNPLUS_PIN,	BTN_UP, true },	// BTNPLUS
	{ GPIOB, BTNMINUS_PIN,	BTN_UP, true },	// BTNMINUS
};

// ----------------------------------------------------------------------------
// LED Defines
//
// LEDs are on GPIOA
#define LED_COUNT	8

// Define LED locations
#define LED1A		0
#define LED1B		1
#define LED2A		2
#define LED2B		3
#define LED3A		4
#define LED3B		5
#define LED4A		6
#define LED4B		7

#define PIN_LED1A	0
#define PIN_LED1B	1
#define PIN_LED2A	12
#define PIN_LED2B	15
#define PIN_LED3A	3
#define PIN_LED3B	4
#define PIN_LED4A	6
#define PIN_LED4B	7

typedef struct {
	GPIO_TypeDef *	Port;
	uint8_t			Pin;
} led_t;

led_t led[LED_COUNT] = {
	{ GPIOA, PIN_LED1A },
	{ GPIOA, PIN_LED1B },
	{ GPIOA, PIN_LED2A },
	{ GPIOA, PIN_LED2B },
	{ GPIOB, PIN_LED3A },
	{ GPIOB, PIN_LED3B },
	{ GPIOB, PIN_LED4A },
	{ GPIOB, PIN_LED4B },
};

#ifdef USE_USART

// ============================================================================
// USART Defines
//
// TX:PA2:Output, RX:PA3:Input
#define USART_GPIO			GPIOA
#define USART_TX_PIN		2
#define USART_RX_PIN		3

#endif	// USE_USART

// ============================================================================

volatile uint32_t curTick;


// ============================================================================


// ============================================================================
// Button interface
// 

// ============================================================================
// Configure a button as input with pullup enabled
void InitButton( int idx )
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

// ============================================================================
// InitButton -- Initialize all the buttons
void configButtons( void )
{
	for ( int idx=0; idx<BTN_COUNT; ++idx ) {
		InitButton( idx );
	}
}

// ============================================================================
// btnCheckState -- Get current state of the specified button
bool btnCheckState( int idx )
{
	btnstate_t newState = (button[idx].Port->IDR & (1<<button[idx].Pin)) ? BTN_UP : BTN_DOWN;
	if ( newState != button[idx].State ) {
		button[idx].Changed = true;
	}
	button[idx].State = newState;

	return button[idx].Changed;
}

// ============================================================================
// PWM interface
//
// Initialize four PWMs, driven by TIM1, with a period of 20mS (50Hz) and a duty
// cycle of 1.5mS
//
// SERVO1 - PA8  - TIM1_CH1	 -	AF2
// SERVO2 - PA9  - TIM1_CH2	 -	AF2
// SERVO3 - PA10 - TIM1_CH3	 -	AF2
// SERVO4 - PA11 - TIM1_CH4	 -	AF2
//
// ============================================================================
//
void InitServoPin( int idx )
{
	uint32_t tmp;

	// Output is push/pull (OTYPER:=0)
	servo[idx].Port->OTYPER &= ~(1<<servo[idx].Pin);

	// Speed is high (OSPEEDR:=3)
	servo[idx].Port->OSPEEDR |= (3<<(servo[idx].Pin*2));

	// No pullup or pulldown (PUPDR:=0)
	servo[idx].Port->PUPDR &= ~(3<<(servo[idx].Pin*2));

	// Set mode to alt function (MODER:=2)
	tmp = servo[idx].Port->MODER & ~(0x03<<(servo[idx].Pin*2));
	servo[idx].Port->MODER = tmp | (2<<(servo[idx].Pin*2));
	
	// Select alternate function
	// NB: AltFunc code is broken for pins 0..7
	servo[idx].Port->AFR[1] &= ~(0x0F<<((servo[idx].Pin-8)*4));
	servo[idx].Port->AFR[1] |= (0x02<<((servo[idx].Pin-8)*4));
}

void InitServo( void )
{
	uint32_t tmp;

	// Enable SERVO_GPIO and TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	for ( int idx=0; idx<SERVO_COUNT; ++idx ) {
		InitServoPin(idx);
	}

	// Initialize Timer for 50Hz (20mS) 
	TIM1->CNT = 0;		// Reset counter
	TIM1->PSC = PWM_PRESCALE;
	TIM1->ARR = PWM_MAX_COUNT;
	TIM1->CCR1 = PWM_MID;
	TIM1->CCR2 = PWM_MID;
	TIM1->CCR3 = PWM_MID;
	TIM1->CCR4 = PWM_MID;
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
					TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	TIM1->CCMR2 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
					TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	TIM1->CCER  |= (TIM_CCER_CC1E | TIM_CCER_CC2E |
					TIM_CCER_CC3E | TIM_CCER_CC4E);
	TIM1->BDTR |= TIM_BDTR_MOE;		// Enable output
	TIM1->CR1 |= TIM_CR1_CEN;		// Start the timer
	TIM1->EGR |= TIM_EGR_UG;		// Force update of shadow registers
}

// ============================================================================
// LED interface
// 
// Initialize LED pins as output with no pullup or pulldown
//	Mode:Output, Type:Push-pull, Speed:low, PuPd:None
void InitLedPin( int idx )
//void InitLedPin( GPIO_TypeDef * GPIO, int Pin )
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
}

// ============================================================================
void InitLED( void )
{
	for ( int idx=0; idx<LED_COUNT; ++idx ) {
		InitLedPin( idx );
	}
}

// ============================================================================
// ledOff -- Set the pin high to turn the LED off
void ledOff( int idx )
{
	led[idx].Port->BSRR = 1<<led[idx].Pin;
}

// ============================================================================
// ledOn -- Set the pin low to turn the LED on
void ledOn( int idx )
{
	led[idx].Port->BRR = 1<<led[idx].Pin;
}
// ============================================================================
// ledToggle -- Toggle a led
void ledToggle( int idx )
{
	led[idx].Port->ODR ^= 1<<led[idx].Pin;
}

#ifdef USE_USART

// ============================================================================
// ============================================================================
// USART - Simple UART interface without interrupts, DMA or buffering
// ============================================================================
// InitUSART -- Initialize USART to baud rate, no parity, 8 data bits, 1 stop bit
void InitUSART( uint32_t baud )
{
	uint32_t tmp;

	// USART_TX_PIN:PA2:Output, USART_RX_PIN:PA3:Input
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;		// USART2 clock on

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

	// Connect USART_TX_PIN:PA2 and PA3 to USART2
	USART_GPIO->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL2);
	USART_GPIO->AFR[0] |= ((1 << (3*4)) | (1 << (2*4)));

	USART2->CR1 = ( USART_CR1_TE | USART_CR1_RE );
	USART2->BRR = (SystemCoreClock/2) / baud;
	USART2->CR1 |= USART_CR1_UE;
}

// ============================================================================
// usartReadByte -- Read a byte from the USART, EOF in no data ready
int usartReadByte( void )
{
	if ( USART2->ISR & USART_ISR_RXNE )
		return (int)USART2->RDR;
	return EOF;
}

// ============================================================================
// usartWriteByte -- Write byte to USART, busy wait, returns char written
int usartWriteByte( uint32_t byte )
{
	while( ! (USART2->ISR & USART_ISR_TXE) )
		;
	USART2->TDR = byte;

	return byte;
}

// ============================================================================
// usartTxEmpty -- Return true if USART transmitter can take another character
int usartTxEmpty()
{
	return (USART2->ISR & USART_ISR_TXE) ? 1 : 0; 
}

#endif	// USE_USART

// ============================================================================
// ============================================================================
// btnCheck -- Change servo/turnout state when the button changes state
void btnCheck( void )
{
	for ( int idx=0; idx<SERVO_COUNT; ++idx ) {
		if ( btnCheckState( idx ) ) {
			button[idx].Changed = false;

			if ( button[idx].State ) {
				ledOn( idx*2 );
				ledOff( idx*2 + 1 );
				servo[idx].targetPos = PWM_MIN;
			}
			else {
				ledOff( idx*2 );
				ledOn( idx*2 + 1 );
				servo[idx].targetPos = PWM_MAX;
			}
		}
	}
}

// ============================================================================
// Handler for the SysTick interrupt
void SysTick_Handler( void )
{
	++curTick;
}


// ============================================================================
int main( void )
{
	int	ch = 0;
	uint32_t ccount = 0;
	uint32_t lastTick;
	int pwm;

	SystemCoreClockUpdate();
	SysTick_Config( SystemCoreClock / HB_HZ);

	// Enable peripheral clocks
	// TODO:	Remove GPIOCEN when moving to the smaller CPU
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN);
	
#ifdef USE_USART
	InitUSART(400);
#endif	// USE_USART

	InitLED();
	InitServo();

	configButtons();

	while( 1 ) {
		if ( curTick > (HB_HZ) ) {
			curTick -= (HB_HZ);

			// Once per second processing...

		}
		if ( curTick != lastTick ) {
			lastTick = curTick;

			// On each timer tick move the turnout slightly closer to the new position
			for ( int idx=0; idx<SERVO_COUNT; ++idx ) {
				if ( servo[idx].currentPos < servo[idx].targetPos ) {
					servo[idx].currentPos += SERVO_DELTA;
					if ( servo[idx].currentPos > servo[idx].targetPos ) {
						servo[idx].currentPos = servo[idx].targetPos;
					}
				}
				else
				if ( servo[idx].currentPos > servo[idx].targetPos ) {
					servo[idx].currentPos -= SERVO_DELTA;
					if ( servo[idx].currentPos < servo[idx].targetPos ) {
						servo[idx].currentPos = servo[idx].targetPos;
					}
				}
			}

			TIM1->CCR1 = servo[SERVO1].currentPos;
			TIM1->CCR2 = servo[SERVO2].currentPos;
			TIM1->CCR3 = servo[SERVO3].currentPos;
			TIM1->CCR4 = servo[SERVO4].currentPos;

			btnCheck();
		}
#ifdef USE_USART
		if ( usartTxEmpty() ) {
			usartWriteByte(ch+33);
			++ch;
			ch &= 0x3F;
		}
#endif	// USE_USART
	}
}

// ============================================================================

