/*
 * wp_nav.c
 *
 *  Created on: 10-Oct-2015
 *      Author: atulya
 */

#include "main.h"

void initializeWPNav()
{
	wp_nav._wp_speed_cms = WPNAV_WP_SPEED;
	wp_nav._wp_radius_cm = WPNAV_WP_RADIUS;
	wp_nav._wp_speed_up_cms = WPNAV_WP_SPEED_UP;
	wp_nav._wp_speed_down_cms = WPNAV_WP_SPEED_DOWN;
	wp_nav._loiter_speed_cms = WPNAV_LOITER_SPEED;
	wp_nav._wp_accel_cms = WPNAV_ACCELERATION;
	wp_nav._wp_accel_z_cms = WPNAV_WP_ACCEL_Z_DEFAULT;
	wp_nav._loiter_jerk_max_cmsss = WPNAV_LOITER_JERK_MAX_DEFAULT;
	wp_nav._loiter_accel_cmss = WPNAV_LOITER_ACCEL;
	wp_nav._loiter_accel_min_cmss = WPNAV_LOITER_ACCEL_MIN;

	wp_nav._loiter_step = 0;
	wp_nav._pilot_accel_fwd_cms = 0;
	wp_nav._pilot_accel_rgt_cms = 0;
	wp_nav._pilot_desired_yaw_rate = 0;
	wp_nav._pilot_desired_climb_rate = 0;
	wp_nav._pilot_max_z_velocity = WPNAV_WP_SPEED_DOWN;
	wp_nav._dt_pilot_inp = PILOT_INPUT_DT_50HZ;
	wp_nav._last_pilot_update_ms = 0;

	initializeVector2fToZero(&wp_nav._loiter_desired_accel);
	initializeVector2fToZero(&wp_nav.waypoint);

	wp_nav._wp_last_update = 0;
	wp_nav._wp_step = 0;
	wp_nav._track_length = 0;
	wp_nav._track_desired = 0.0f;
	wp_nav._limited_speed_xy_cms = 0.0f;
	wp_nav._track_accel = 0.0f;
	wp_nav._track_speed = 0.0f;
	wp_nav._track_leash_length = 0.0f;
	wp_nav._slow_down_dist = 0.0f;
	wp_nav._spline_time = 0.0f;
	wp_nav._spline_time_scale = 0.0f;
	wp_nav._spline_vel_scaler = 0.0f;
	wp_nav._yaw = 0.0f;

	// init flags
	wp_nav._flags.reached_destination = 0;
	wp_nav._flags.fast_waypoint = 0;
	wp_nav._flags.slowing_down = 0;
	wp_nav._flags.recalc_wp_leash = 0;
	wp_nav._flags.new_wp_destination = 0;

	initializeLPF(&wp_nav.channel6_filter);
	wp_nav.channel6_filter.cutoff_freq = 0.4;
}

void loiter_run()
{
	uint32_t now = millis();
	float dt = now - wp_nav._last_pilot_update_ms;

	// run at pilot_input update rate.
	if (dt >= wp_nav._dt_pilot_inp)
	{
		// sanity check dt
		if (dt >= 0.2f) {
			dt = 0.0f;
		}
//		getNavDesiredAcceleration();
		getPilotDesiredAcceleration();
		getPilotDesiredYawRate();
		getPilotClimbRate();
	}
	updateLoiter();
	setAttitude(pos_control.roll_target, pos_control.pitch_target, wp_nav._pilot_desired_yaw_rate);

	updateAltHold();
	// send throttle to attitude controller with angle boost
	setThrottleOut(pos_control.throttle_in, 1);

	//set target altitude based on the desired climb rate
}

void getNavDesiredAcceleration()
{
	Vector2f desired_accel;
	desired_accel.x = pow(OMEGA,2)*(wp_nav.waypoint.x - pos_control.pos_target.x) -2*TAU*pos_control.vel_desired.x;
	desired_accel.y = pow(OMEGA,2)*(wp_nav.waypoint.y - pos_control.pos_target.y) -2*TAU*pos_control.vel_desired.y;

	wp_nav._pilot_accel_fwd_cms =  desired_accel.x*ahrs.cos_psi + desired_accel.y*ahrs.sin_psi;
	wp_nav._pilot_accel_rgt_cms = -desired_accel.x*ahrs.sin_psi + desired_accel.y*ahrs.cos_psi;

}

void getPilotDesiredAcceleration()
{
	int16_t control_pitch_in = constrain_int(rc_in[1], STICK_MIN, STICK_MAX);
	int16_t control_roll_in = constrain_int(rc_in[0], STICK_MIN, STICK_MAX);

	int16_t control_pitch = (control_pitch_in - STICK_MID);
	int16_t control_roll = (control_roll_in - STICK_MID);
	//TODO check the above for errors

	if(abs(control_pitch) < STICK_DEADBAND)
		control_pitch = 0;
	if(abs(control_roll) < STICK_DEADBAND)
		control_roll = 0;

	control_pitch = 0;	// TODO override added to check without remote
	control_roll = 0;

	wp_nav._pilot_accel_fwd_cms = -control_pitch * wp_nav._loiter_accel_cmss / ((STICK_MAX-STICK_MIN)/2);
	wp_nav._pilot_accel_rgt_cms = control_roll * wp_nav._loiter_accel_cmss / ((STICK_MAX-STICK_MIN)/2);
}

void getPilotDesiredYawRate()
{
	//TODO check these formulae for errors
	int16_t control_yaw_rate = constrain_int(rc_in[3], STICK_MIN, STICK_MAX);
	wp_nav._pilot_desired_yaw_rate = (control_yaw_rate - STICK_MID)*STICK_TO_DEGREEPS;

	wp_nav._pilot_desired_yaw_rate = 0; //TODO override added to check without remote
}

void getPilotClimbRate()
{
	//TODO check these formulae for errors
	float desired_rate;

	float deadband_top = MID_STICK_THROTTLE + THROTTLE_DEADZONE;
	float deadband_bottom = MID_STICK_THROTTLE - THROTTLE_DEADZONE;

	// ensure a reasonable throttle value
	float throttle_control = constrain_float(rc_in[2],THROTTLE_MIN,THROTTLE_MAX);

	throttle_control = constrain_float(MID_STICK_THROTTLE,THROTTLE_MIN,THROTTLE_MAX); //TODO override added to check without remote

	// check throttle is above, below or in the deadband
	if (throttle_control < deadband_bottom)
	{
		// below the deadband
		desired_rate = wp_nav._pilot_max_z_velocity * (throttle_control-deadband_bottom) / (deadband_bottom-THROTTLE_MIN);
	}else
		if (throttle_control > deadband_top)
	        desired_rate = wp_nav._pilot_max_z_velocity * (throttle_control-deadband_top) / (THROTTLE_MAX-deadband_top);
	    else{
	        // must be in the deadband
	        desired_rate = 0.0f;
	    }

	wp_nav._pilot_desired_climb_rate = desired_rate;

}

static void calcLoiterDesiredVelocity(float nav_dt)
{
	// calculate a loiter speed limit which is the minimum of the value set by the WPNAV_LOITER_SPEED
	// parameter and the value set by the EKF to observe optical flow limits

	// range check nav_dt
	if( nav_dt < 0 )
		return;

	pos_control.speed_cms = wp_nav._loiter_speed_cms;
	pos_control.accel_cms = wp_nav._loiter_accel_cmss;

	// rotate pilot input to lat/lon frame
	Vector2f desired_accel;
	desired_accel.x = (wp_nav._pilot_accel_fwd_cms*ahrs.cos_psi - wp_nav._pilot_accel_rgt_cms*ahrs.sin_psi);
	desired_accel.y = (wp_nav._pilot_accel_fwd_cms*ahrs.sin_psi + wp_nav._pilot_accel_rgt_cms*ahrs.cos_psi);

	// calculate the difference
	Vector2f des_accel_diff;
	des_accel_diff.x = (desired_accel.x - wp_nav._loiter_desired_accel.x);
	des_accel_diff.y = (desired_accel.y - wp_nav._loiter_desired_accel.y);

	// constrain and scale the desired acceleration
	float des_accel_change_total = pythagorous2(des_accel_diff.x, des_accel_diff.y);
	float accel_change_max = wp_nav._loiter_jerk_max_cmsss * nav_dt;

	if (wp_nav._loiter_jerk_max_cmsss > 0.0f && des_accel_change_total > accel_change_max && des_accel_change_total > 0.0f)
	{
		des_accel_diff.x = accel_change_max * des_accel_diff.x/des_accel_change_total;
		des_accel_diff.y = accel_change_max * des_accel_diff.y/des_accel_change_total;
	}

	// adjust the desired acceleration
	wp_nav._loiter_desired_accel.x += des_accel_diff.x;
	wp_nav._loiter_desired_accel.y += des_accel_diff.y;

//	debug("calcLoiterDesVel: dt: %f; exp dt :%f; accelX: %f; accelY: %f", nav_dt, pos_control.dt_xy, wp_nav._loiter_desired_accel.x,
//			wp_nav._loiter_desired_accel.y);

	// get pos_control's feed forward velocity
	Vector3f desired_vel = pos_control.vel_desired;

	// add pilot commanded acceleration
	desired_vel.x += wp_nav._loiter_desired_accel.x * nav_dt;
	desired_vel.y += wp_nav._loiter_desired_accel.y * nav_dt;

	    //NOT REQUIRED TO INCLUDE THIS for a simple controller
//	    float desired_speed = pythagorous2(desired_vel.x, desired_vel.y);
//
//	    if (desired_speed != 0)
//	    {
//	        Vector2f desired_vel_norm;
//	        desired_vel_norm.x = desired_vel.x/desired_speed;
//	        desired_vel_norm.y = desired_vel.y/desired_speed;
//
//	        float drag_speed_delta = -_loiter_accel_cmss*nav_dt*desired_speed/gnd_speed_limit_cms;
//
//	        if (wp_nav.pilot_accel_fwd_cms == 0 && _pilot_accel_rgt_cms == 0) {
//	            drag_speed_delta = min(drag_speed_delta,-_loiter_accel_min_cmss*nav_dt);
//	        }
//
//	        desired_speed = max(desired_speed+drag_speed_delta,0.0f);
//	        desired_vel = desired_vel_norm*desired_speed;
//	    }

	// Apply EKF limit to desired velocity -  this limit is calculated by the EKF and adjusted as required to ensure certain sensor limits are respected (eg optical flow sensing)
	float horizSpdDem = pythagorous2(desired_vel.x, desired_vel.y);
	if (horizSpdDem > pos_control.speed_cms)
	{
		desired_vel.x = desired_vel.x * pos_control.speed_cms / horizSpdDem;
		desired_vel.y = desired_vel.y * pos_control.speed_cms / horizSpdDem;
	}

	// send adjusted feed forward velocity back to position controller
	pos_control.vel_desired.x = desired_vel.x;
	pos_control.vel_desired.y = desired_vel.y;

}

void updateLoiter()
{
	// calculate dt
	float dt = (millis() - pos_control.last_update_xy_ms)*0.001f;

	// run at poscontrol update rate.
	// TODO: (something present on original code)run on user input to reduce latency, maybe if (user_input || dt >= _pos_control.get_dt_xy())
	if (dt >= pos_control.dt_xy)
	{
		// sanity check dt
		if (dt >= 0.2f) {
			dt = 0.0f;
		}
		calcLoiterDesiredVelocity(dt);
		updateXYController(XY_MODE_POS_LIMITED_AND_VEL_FF, 1);//TODO check out which mode is better
	}
}

void updateAltHold()
{
	float dt = (millis() - pos_control.last_update_z_ms)*0.001f;

	// run at poscontrol update rate.
	// TODO: (something present on original code)run on user input to reduce latency, maybe if (user_input || dt >= _pos_control.get_dt_xy())
	if (dt >= pos_control.dt)
	{
		// sanity check dt
		if (dt >= 0.2f) {
			dt = 0.0f;
		}

		setAltTargetfromClimbRate(wp_nav._pilot_desired_climb_rate, pos_control.dt);
		updateZController();
	}
}
