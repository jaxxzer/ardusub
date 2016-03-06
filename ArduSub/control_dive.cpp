/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"
#define DIVE_TRANSITION 0.10 //10cm above/below dive target, transition speed for smoothness
#define DIVE_TARGET -0.30 //30cm depth is dive target

static bool dive_with_gps;

static uint32_t dive_start_time;
static bool dive_pause;

// dive_init - initialise dive controller
bool Sub::dive_init(bool ignore_checks)
{
    // check if we have GPS and decide which dive we're going to do
    dive_with_gps = position_ok();
    if (dive_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    dive_start_time = millis();

    dive_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    return true;
}

// dive_run - runs the dive controller
// should be called at 100hz or more
void Sub::dive_run()
{
    if (dive_with_gps) {
        dive_gps_run();
    }else{
        dive_nogps_run();
    }
}

// dive_run - runs the dive controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Sub::dive_gps_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

#if PRECISION_LANDING == ENABLED
    // run precision landing
    if (!ap.land_repo_active) {
        wp_nav.shift_loiter_target(precland.get_target_shift(wp_nav.get_loiter_target()));
    }
#endif

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < 4000) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

// dive_nogps_run - runs the dive controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Sub::dive_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    //ToDo SUB: remove these checks, except ap.at_surface

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.at_surface || !motors.get_interlock()) {
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.at_surface && (ap.throttle_zero || failsafe.radio)) {
            //init_disarm_motors();
        	set_mode(ALT_HOLD)
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.at_surface) {
            //init_disarm_motors();
        	set_mode(ALT_HOLD);
        }
#endif
        return;
    }

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // pause 4 seconds before beginning dive descent
    float cmb_rate;
    if(dive_pause && millis()-dive_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    } else {
        dive_pause = false;
        cmb_rate = get_dive_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

// get_dive_descent_speed - high level diving logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
float Sub::get_dive_descent_speed()
{
#if CONFIG_SONAR == ENABLED
    bool sonar_ok = sonar_enabled && (sonar.status() == RangeFinder::RangeFinder_Good);
#else
    bool sonar_ok = false;
#endif
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (pos_control.get_pos_target().z >= pv_alt_above_origin(DIVE_TARGET + DIVE_TRANSITION) && !(sonar_ok && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}

// dive_do_not_use_GPS - forces dive-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in DIVE mode that we do not use the GPS
//  has no effect if we are not already in DIVE mode
void Sub::dive_do_not_use_GPS()
{
    dive_with_gps = false;
}

// set_mode_dive_with_pause - sets mode to DIVE and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Sub::set_mode_dive_with_pause()
{
    set_mode(DIVE);
    dive_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// diving_with_GPS - returns true if vehicle is diving using GPS
bool Sub::diving_with_GPS() {
    return (control_mode == DIVE && dive_with_gps);
}
