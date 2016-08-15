/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// ArduSub position hold flight mode
// Code by Jacob Walser

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED
namespace {
	static uint32_t last_poshold_message_ms = 0;

}

// poshold_init - initialise PosHold controller
bool Sub::poshold_init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    
    follow_bottom = false;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());


	// set target to current position
	// only init here as we can switch to PosHold in flight with a velocity <> 0 that will be used as _last_vel in PosControl and never updated again as we inhibit Reset_I
	wp_nav.init_loiter_target();

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
	uint32_t tnow = millis();

    const Vector3f& vel = inertial_nav.get_velocity();

    // convert inertial nav earth-frame velocities to body-frame
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    float vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
    float vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
    	follow_bottom = false;
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-motors.get_throttle_hover());
        return;
    }


	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();

	// set motors to full range
	motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

	// run loiter controller
	wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
	target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

	int16_t pilot_lateral = channel_lateral->get_control_in();
	int16_t pilot_forward = channel_forward->get_control_in();

	// get poshold forward and lateral outputs from wp_nav pitch and roll (from copter code)
	int32_t poshold_lateral = wp_nav.get_roll();
	int32_t poshold_forward = -wp_nav.get_pitch(); // output is reversed

	// constrain target forward/lateral values
	poshold_lateral = constrain_int16(poshold_lateral, -aparm.angle_max, aparm.angle_max);
	poshold_forward = constrain_int16(poshold_forward, -aparm.angle_max, aparm.angle_max);

	float lateral_out = 0;
	float forward_out = 0;

	if(pilot_lateral > 1000 || pilot_lateral < -1000 || pilot_forward > 1000 || pilot_forward < -1000) {
		lateral_out = (float)pilot_lateral/(float)aparm.angle_max;
		forward_out = (float)pilot_forward/(float)aparm.angle_max;
		wp_nav.init_loiter_target();
	} else {
		lateral_out = (float)poshold_lateral/(float)aparm.angle_max;
		forward_out = (float)poshold_forward/(float)aparm.angle_max;
	}

	motors.set_lateral(lateral_out);
	motors.set_forward(forward_out);

	//////////
	// Get real roll/pitch inputs and apply them
	//////////

	// convert pilot input to lean angles
	// To-Do: convert get_pilot_desired_lean_angles to return angles as floats
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

	// update attitude controller targets
	attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	/////////
	/////////

	// adjust climb rate using rangefinder
	if (rangefinder_alt_ok() && follow_bottom) {
		// if rangefinder is ok, use surface tracking
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
	}

	// update altitude target and call position controller
	pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
	pos_control.update_z_controller();


	if(tnow > last_poshold_message_ms + 200) {

		last_poshold_message_ms = tnow;
		mavlink_msg_command_long_send(
				(mavlink_channel_t)0, //channel
				0, //target system
				0, //target component
				45, //command
				0, //confirmation
				channel_forward->get_control_in(),//1
				channel_lateral->get_control_in(),
				channel_throttle->get_control_in(),
				channel_yaw->get_control_in(),
				channel_roll->get_control_in(),
				channel_pitch->get_control_in(),
				0
				);
	}
}
#endif  // POSHOLD_ENABLED == ENABLED
