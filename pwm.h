// =============================================================================
//
#ifndef __PWM_H__

// =============================================================================
// Servo Defines
// =============================================================================
// Available PWM outputs. This program uses TIM1 channels 1..4 on PA8..PA11
//
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
// 
// =============================================================================
// Initialize four PWMs, driven by TIM1, with a period of 20mS (50Hz) and a
// nominal duty cycle of 1.5mS
//
// SERVO1 - PA8  - TIM1_CH1	 -	AF2
// SERVO2 - PA9  - TIM1_CH2	 -	AF2
// SERVO3 - PA10 - TIM1_CH3	 -	AF2
// SERVO4 - PA11 - TIM1_CH4	 -	AF2
//
// =============================================================================
//
// There are 4 servos
enum eServo { SERVO1, SERVO2, SERVO3, SERVO4 };
#define SERVO_COUNT		4
//
// GPIO pins for the servos
#define	SERVO1_PIN		 8
#define	SERVO2_PIN		 9
#define	SERVO3_PIN		10
#define	SERVO4_PIN		11
//
// Maximum and minimum PWM values
// PWM_MIN - Min allowed servo position, absolute minimum is approx 200
#define PWM_MIN			1150	// Clockwise when looking at shaft
//
// PWM_MAX - Max allowed servo position, absolute maximum is about 2280
#define PWM_MAX			1700
//
// Midpoint of the servo range
#define PWM_MID			((PWM_MIN+PWM_MAX)/2)
//
// Prescale applied to system clock to allow the PWM timer to run at PWM_TIMER_HZ
#define PWM_PRESCALE	(48-1)
//
// Timer clock speed for the timer driving the servo subsystem 
#define PWM_TIMER_HZ	1000000UL
//
// SERVO_HZ - How often a servo pulse is sent
#define SERVO_HZ		50L
//
// PWM pulses per second on each servo ( 1,000,000 / 50 )
#define PWM_HZ			(PWM_TIMER_HZ)/(PWM_MAX_COUNT)
//
// PWM_MAX_COUNT - Number of PWM timer period is 20mS (20,000uS)
#define PWM_MAX_COUNT	(PWM_TIMER_HZ)/(SERVO_HZ)
//
// PWM speed -- number of seconds to go from PWM_MIN to PWM_MAX
#define PWM_SPEED		3
//
// How much to increment(decrement) servo position for each servo timer tic
//						(1700    - 1150 ) / ( 5 * 50 )
#define SERVO_DELTA		(PWM_MAX-PWM_MIN)/(PWM_SPEED*PWM_HZ)
//
typedef struct {
	GPIO_TypeDef *	Port;		// GPIO port for this servo
	uint8_t			Pin;		// GPIO pin for this servo
	uint16_t		currentPos;	// Current position
	uint16_t		targetPos;	// Final position at end of servo move		Backed in EEPROM
	uint16_t		minPos;		// Servo position in minimum state			Backed in EEPROM
	uint16_t		maxPos;		// Servo position in maximum state			Backed in EEPROM
} servo_t;
//
servo_t servo[SERVO_COUNT];	
//
extern void servoInit( void );		// servoInit -- Initialize pwm subsystem to drive the servos
extern void servoMove( void );		// servoMove -- Move the servo slightly closer to the target position

#endif	// __PWM_H__
//
// ==============================================================================
