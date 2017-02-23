// =============================================================================
//
#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
//
#include "servo.h"
#include "pwm.h"
#include "led.h"
#include "button.h"

// =============================================================================
// Servo Data
// =============================================================================
servo_t servo[SERVO_COUNT] = {
	{ GPIOA,	SERVO1_PIN, (PWM_MID), (PWM_MID), PWM_MIN, PWM_MAX },
	{ GPIOA,	SERVO2_PIN, (PWM_MID), (PWM_MID), PWM_MIN, PWM_MAX },
	{ GPIOA,	SERVO3_PIN, (PWM_MID), (PWM_MID), PWM_MIN, PWM_MAX },
	{ GPIOA,	SERVO4_PIN, (PWM_MID), (PWM_MID), PWM_MIN, PWM_MAX },
};
//
// ==============================================================================
// PWM interface
// ==============================================================================
//
void servoConfigPin( enum eServo idx )
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
	// NB: The alternate function code below will NOT work for pins 0..7
	servo[idx].Port->AFR[1] &= ~(0x0F<<((servo[idx].Pin-8)*4));
	servo[idx].Port->AFR[1] |= (0x02<<((servo[idx].Pin-8)*4));
}
//
// ==============================================================================
//
void servoInit( void )
{
	// Enable SERVO_GPIO and TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	for ( int idx=0; idx<SERVO_COUNT; ++idx ) {
		servoConfigPin( (enum eServo)idx );
	}

	// Initialize Timer for 50Hz (20mS) 
	//
	TIM1->CNT = 0;		// Reset counter register
	//
	TIM1->PSC = PWM_PRESCALE;	// Set clock prescale
	//
	TIM1->ARR = PWM_MAX_COUNT;	// Auto Reload Register
	//
//	TIM1->CCR1 = PWM_MID;		// Capture/Compare values
//	TIM1->CCR2 = PWM_MID;
//	TIM1->CCR3 = PWM_MID;
//	TIM1->CCR4 = PWM_MID;

	TIM1->CCR1 = 0;		// Capture/Compare values
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	// Configure all channels for output (Compare mode):
	//	CCMRx.OCxM  = 0b110 : PWM Mode 1
	//	CCMRx.CCxS  = 0b00  : Channel is output
	//	CCMRx.OCxPE = 0b1   : Preload from TIMx_CCRx
	//	CCMRx.OCxCE = 0b0	: Clear Enable
	//	CCMRx.OCxFE = 0b0	: Fast Enable
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
					TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	//
	TIM1->CCMR2 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
					TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	// CCER.CCxE  = 0b1 : Enable Capture/Compare on channel x
	// CCER.CCxP  = 0x0 : OCx is active high
	// CCER.CCxNP = 0x0 : OCxN is active high
	// CCER.CCxNE = 0x0 : Complementary output (OCxN) is disabled
	TIM1->CCER  |= (TIM_CCER_CC1E | TIM_CCER_CC2E |
					TIM_CCER_CC3E | TIM_CCER_CC4E);
	//
	TIM1->BDTR |= TIM_BDTR_MOE;		// Enable output
	//
	// Edge aligned, up count, unbuffered ARR, NOT one pulse, update on overflow,
	// update enabled, counter enabled
	TIM1->CR1 |= TIM_CR1_CEN;		// Start the timer
	//
	TIM1->EGR |= TIM_EGR_UG;		// Force update of shadow registers
}
//
// ==============================================================================
// servoSetCurrent -- Read servo buttons and assign servo positions
//
void servoSetCurrent( void )
{
	enum eServo idx;

	for ( idx=SERVO1; idx<=SERVO4; ++idx ) {
		// The real currentPos is unknowable, set it to the targetPos
		servo[idx].targetPos = servo[idx].currentPos = btnIsDown( idx ) ? PWM_MIN : PWM_MAX;
	}
}
//
// ==============================================================================
// servoMove -- Move the servo closer to the target position by an increment
//
void servoMove( void )
{
	// Move the turnout slightly closer to the new position
	for ( int idx=0; idx<SERVO_COUNT; ++idx ) {
		if ( servo[idx].currentPos < servo[idx].targetPos ) {
			servo[idx].currentPos += SERVO_DELTA;
			// Check and fix any overshoot
			if ( servo[idx].currentPos > servo[idx].targetPos ) {
				servo[idx].currentPos = servo[idx].targetPos;
			}
		}
		else if ( servo[idx].currentPos > servo[idx].targetPos ) {
			servo[idx].currentPos -= SERVO_DELTA;
			// Check and fix any undershoot
			if ( servo[idx].currentPos < servo[idx].targetPos ) {
				servo[idx].currentPos = servo[idx].targetPos;
			}
		}
		//
		// If the servo is at the target position turn on one of the LEDs
		ledOff( idx*2 );
		ledOff( idx*2 + 1 );
		if ( servo[idx].currentPos == servo[idx].minPos ) {
			ledOn( idx*2 + 1 );
		}
		else if ( servo[idx].currentPos == servo[idx].maxPos ) {
			ledOn( idx*2 );
		}
	}

	TIM1->CCR1 = servo[SERVO1].currentPos;
	TIM1->CCR2 = servo[SERVO2].currentPos;
	TIM1->CCR3 = servo[SERVO3].currentPos;
	TIM1->CCR4 = servo[SERVO4].currentPos;
}
//
// ==============================================================================
