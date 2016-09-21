/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Code by Jacob Walser: jwalser90@gmail.com

#include "Sub.h"

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;

// checks if we have have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
// ToDo: doesn't need to be called this fast
void Sub::update_surface_and_bottom_detector()
{
	if(!motors.armed()) { // only update when armed
		set_surfaced(false);
		set_bottomed(false);
		return;
	}

	// Information is only used in auto depth enabled modes, which are locked out if no depth sensor present
	if(!ap.depth_sensor_present) {
		return;
	}

	current_depth = barometer.get_altitude() * 100; // cm

	// get velocity in cm/s
	Vector3f velocity = inertial_nav.get_velocity();

	// check that we are not moving up or down
	bool vel_stationary = velocity.z > -2 && velocity.z < 2;

	if (vel_stationary) {
		if(motors.limit.throttle_upper || (pos_control.get_alt_target() - current_depth) > 2) {
			// surface criteria met, increment counter and see if we've triggered
			if( surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				surface_detector_count++;
			} else {
				set_bottomed(false);
				set_surfaced(true);
			}

		} else if(motors.limit.throttle_lower || (pos_control.get_alt_target() - current_depth) < -2) {
			// bottom criteria met, increment counter and see if we've triggered
			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				bottom_detector_count++;
			} else {
				set_surfaced(false);
				set_bottomed(true);
			}

		} else { // we're not at the limits of throttle, so reset both detectors
			set_surfaced(false);
			set_bottomed(false);
		}

	} else { // we're moving up or down, so reset both detectors
		set_surfaced(false);
		set_bottomed(false);
	}
}

void Sub::set_surfaced(bool at_surface) {


	if(ap.at_surface == at_surface) { // do nothing if state unchanged
		return;
	}

	ap.at_surface = at_surface;

	if(!ap.at_surface) {
		Log_Write_Event(DATA_SURFACED);
		gcs_send_text(MAV_SEVERITY_INFO, "Off Surface");
	} else {
		// make sure we dont set ceiling so low in water that the vehicle can no longer come close to the surface
		// this check is in case our motors go to the limits because we are stuck against the bottom of a boat, or otherwise
		// hung up/snagged at depth
		if(is_zero(barometer.get_baro_drift_offset()) && barometer.get_altitude() * 100 > -30) {
			gcs_send_text(MAV_SEVERITY_INFO, "Setting barometer offset");
			barometer.set_baro_drift_altitude(-barometer.get_altitude()); // offset barometer to read zero here
			//barometer.set_ground_pressure();
		}
		surface_detector_count = 0;
		Log_Write_Event(DATA_NOT_SURFACED);
		gcs_send_text(MAV_SEVERITY_INFO, "Surfaced");
	}
}

void Sub::set_bottomed(bool at_bottom) {

	if(ap.at_bottom == at_bottom) { // do nothing if state unchanged
		return;
	}

	ap.at_bottom = at_bottom;

	if(!ap.at_bottom) {
		Log_Write_Event(DATA_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_INFO, "Off Bottom");
	} else {
		bottom_detector_count = 0;
		Log_Write_Event(DATA_NOT_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_INFO, "Bottomed");
	}
}
