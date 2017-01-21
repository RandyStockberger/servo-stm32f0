// =============================================================================
//
//#define USE_USART
//
//#include "stm32f0xx.h"
//
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdarg.h>
//#include "servo.h"
//#include "button.h"
//#include "led.h"
//#include "pwm.h"
//#include "usart.h"
//
// =============================================================================
//
// Remap LED outputs to not interfer with SWD I/O
#define AVOID_SWD	1

#define HB_HZ		1000
#define HB_50HZ		((HB_HZ)/50)
//
// ==============================================================================
//
//extern volatile uint32_t	curTick;
//
// ==============================================================================
