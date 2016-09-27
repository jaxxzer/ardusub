/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
	// put your initialisation code here
	// this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
	// put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Sub::userhook_50Hz()
{
	// put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Sub::userhook_MediumLoop()
{
	// put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Sub::userhook_SlowLoop()
{
	// put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Sub::userhook_SuperSlowLoop()
{
//	static uint32_t last_msg_ms = 0;
//	if(AP_HAL::millis() > last_msg_ms + 10000) {
//		last_msg_ms = AP_HAL::millis();
//	}
}
#endif
