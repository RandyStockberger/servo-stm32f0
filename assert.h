
#ifndef assert_param

#ifdef USE_ASSERTS

#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

void assert_failed( uint8_t file, uint32_t line )
{
	while(1) {
	}
}

#else	// USE_ASSERTS

#define assert_param(expr) ((void)0)

#endif	// USE_ASSERTS

#endif	// assert_param
