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
//			mavlink_msg_command_long_send(
//					(mavlink_channel_t)0, //channel
//					0, //target system
//					0, //target component
//					51, //command
//					0, //confirmation
//					channel_throttle->get_radio_in(),
//					channel_throttle->get_control_in(),
//					channel_throttle->pwm_to_angle(),
//					channel_throttle->norm_input(),//1
//					channel_throttle->norm_input_dz(),
//					get_throttle_control_dz(),
//					get_pilot_desired_throttle(channel_throttle->get_control_in()));
//

}
#endif
