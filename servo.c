// =============================================================================
//
// TODO:
//	KiCad: Fix diode and inductor footprints
//	KiCad: Add reset button
//	KiCad: Add MOSFET switch to 5V line to each servo
//	KiCad: Remove unused circuitry (DCC decode, Crystal, etc)
//	KiCad: Add circuitry to reverse frog polarity
//
//	KiCad/Software: Add a debug LED
//	KiCad/Software: Remove LEDs from SWDIO and SWDCLK to allow debugger to work
//					and reorganize LED and Button port allocation
//
// =============================================================================
//
// // 												  (Nucleo)
// Pin Name		Function		STM32F030xx			STM32F072RB
// 
// VDD			SYSTEM				 1						 CN7:5
// PF0-OSC_IN						 2				5		 CN7:29		2
// PF1-OSC_OUT						 3				6		 CN7:31		3
// NRST			SYSTEM				 4				7		 CN7:14		4 SYS
// VDDA			SYSTEM				 5				13		CN10:7		5 SYS
// PA0			LED1A				 6				14		 CN7:28		6 L
// PA1			LED1B				 7				15		 CN7:30		7 L
// PA2			TX					 8				16		CN10:35		8
// PA3			RX					 9				17		CN10:37		9
// PA4			BTN1				10				20		 CN7:32		10 B
// PA5			BTN2/Nucleo LED		11				21		CN10:11		11 B
// PA6			BTN3				12				22		CN10:13		12 B
// PA7			BTN4				13				23		CN10:15
// PB0			BTNPLUS				14				26		 CN7:34
// PB1			BTNMINUS			15				27		CN10:24		14 B
// VSS			SYSTEM				16				31		 CN7:8		15 SYS
// VDD			SYSTEM				17				32		 CN7:15		16 SYS
// PA8			SERVO1				18				41		CN10:23
// PA9			SERVO2				19				42		CN10:21		17 S1
// PA10			SERVO3				20				43		CN10:33		18 S2
// PA11			SERVO4				21				44		CN10:14
// PA12			LED2A				22				45		CN10:12
// PA13			LED2B/SWDIO			23				46		 CN7:13		19 SWD
// PA14			LED3A/SWCLK			24				49		 CN7:15		20 SWC
// PA15			LED3B				25				50		 CN7:17
// PB3			???	(LED2B-new)		26				55		CN10:31
// PB4			???	(LED3A-new)		27				56		CN10:27
// PB5			DCCIN				28				57		CN10:29
// PB6			LED4A				29				58		CN10:17
// PB7			LED4B				30				59		 CN7:21
// BOOT0		SYSTEM				31				60		 CN7:7		1 SYS
// VSS			SYSTEM				32				63		CN10:20
// 
// =============================================================================
//
// // 												  (Nucleo)
// Pin Name		Function		STM32F030xx			STM32F072RB
// 
// VDD			SYSTEM				 1						 CN7:5
// PF0-OSC_IN	-					 2				5		 CN7:29
// PF1-OSC_OUT	-					 3				6		 CN7:31
// NRST			SYSTEM				 4				7		 CN7:14
// VDDA			SYSTEM				 5				13		CN10:7
// PA0			LED1A				 6				14		 CN7:28
// PA1			LED1B				 7				15		 CN7:30
// PA2			TX					 8				16		CN10:35
// PA3			RX					 9				17		CN10:37
// PA4			LED2A				10				20		 CN7:32
// PA5			LED2B/Nucleo LED	11				21		CN10:11
// PA6			LED3A				12				22		CN10:13
// PA7			LED3B				13				23		CN10:15
// PB0			BTNPLUS				14				26		 CN7:34
// PB1			BTNMINUS			15				27		CN10:24
// VSS			SYSTEM				16				31		 CN7:8
// VDD			SYSTEM				17				32		 CN7:15
// PA8			SERVO1				18				41		CN10:23
// PA9			SERVO2				19				42		CN10:21
// PA10			SERVO3				20				43		CN10:33
// PA11			SERVO4				21				44		CN10:14
// PA12			LED4A				22				45		CN10:12
// PA13			SWDIO				23				46		 CN7:13
// PA14			SWCLK				24				49		 CN7:15
// PA15			LED4B				25				50		 CN7:17
// PB3			BTN1				26				55		CN10:31
// PB4			BTN2				27				56		CN10:27
// PB5			-					28				57		CN10:29
// PB6			BTN3				29				58		CN10:17
// PB7			BTN4				30				59		 CN7:21
// BOOT0		SYSTEM				31				60		 CN7:7
// VSS			SYSTEM				32				63		CN10:20
// 
// =============================================================================
// PB0			BTNPLUS				14				26		 CN7:34
// PB1			BTNMINUS			15				27		CN10:24
// PB3			BTN1				26				55		CN10:31
// PB4			BTN2				27				56		CN10:27
// PB6			BTN3				29				58		CN10:17
// PB7			BTN4				30				59		 CN7:21
// =============================================================================
// PA0			LED1A				 6				14		 CN7:28
// PA1			LED1B				 7				15		 CN7:30
// PA4			LED2A				10				20		 CN7:32
// PA5			LED2B/Nucleo LED	11				21		CN10:11
// PA6			LED3A				12				22		CN10:13
// PA7			LED3B				13				23		CN10:15
// PA12			LED4A				22				45		CN10:12
// PA15			LED4B				25				50		 CN7:17
//
#include "stm32f0xx.h"
//
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "servo.h"
#include "button.h"
#include "led.h"
#include "pwm.h"
//
// =============================================================================
//
volatile uint32_t	curTick;
//
// ==============================================================================
// Handler for the SysTick interrupt
void SysTick_Handler( void )
{
	++curTick;
}
//
//	extern uint16_t EE_Init(void);
//	uint16_t VirtAddVarTab[3] = { 1, 2, 3 };
// ==============================================================================
//
int main( void )
{
	uint32_t lastTick;

	SystemCoreClockUpdate();
	SysTick_Config( SystemCoreClock / HB_HZ);

	// Enable peripheral clocks
#ifdef STM32F030x6
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN );
#endif	// STM32F030x6
#ifdef STM32F072xB
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN);
#endif	// STM32F072xB
	
#ifdef USE_USART
	usartInit(9600);
#endif	// USE_USART

//	EE_Init();
	ledInit();
	btnInit();
	servoInit();
	
	while( 1 ) {
		if ( lastTick != curTick ) {
			//
			// 1000Hz processing
			btnISRProc();

			if ( curTick >= HB_50HZ ) {
				curTick -= HB_50HZ;
				//
				// 50Hz processing
				servoMove();	// Adjust the servo position
			}
			lastTick = curTick;
		}
	}
}
//
// ==============================================================================
