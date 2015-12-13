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
	wp_nav._pilot_max_xy_speed = WPNAV_WP_SPEED_MIN;
	wp_nav._dt_pilot_inp = PILOT_INPUT_DT_50HZ;
	wp_nav._last_pilot_update_ms = 0;

	initializeVector2fToZero(&wp_nav._loiter_desired_accel);

	initializeVector2fToZero(&wp_nav.waypoint);
	wp_nav.flag_auto_wp_enable = 0;
	wp_nav.flag_waypoint_received = 0;
	wp_nav.count_wp_enable = 0;

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

	initializeLPF(&wp_nav.channel6_filter, 0.8);
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

		checkSticksForAutoWPNav();

		//initiate AUTO_WPNAV mode if sticks have been idle and a new waypoint is available
		if(wp_nav.count_wp_enable == AUTO_WPNAV_COUNT_THRESHOLD && wp_nav.flag_waypoint_received == 1)
		{
			wp_nav.flag_waypoint_received = 0;
			wp_nav.flag_auto_wp_enable = 1;
		}

		//DISABLE AUTO_WPNAV mode if sticks have been disturbed
		// (if previously it was enabled it will be disabled in AUTO_WPNAV_COUNT_THRESHOLD/4*0.02 seconds)
		if(wp_nav.count_wp_enable < AUTO_WPNAV_COUNT_THRESHOLD*3.0/4)
			wp_nav.flag_auto_wp_enable = 0;

		if(wp_nav.flag_auto_wp_enable == 1)
		{
			getWPNavDesiredVelocity();
		}
		else
		{
			getPilotDesiredXYVelocity();
//			getPilotDesiredAcceleration();
		}

		getPilotDesiredYawRate();
		getPilotClimbRate();

		wp_nav._last_pilot_update_ms = now;
	}
	updateLoiter();
	setAttitude(pos_control.roll_target, pos_control.pitch_target, wp_nav._pilot_desired_yaw_rate);

	//Alt hold control code
	//set target altitude based on the desired climb rate
	setAltTargetfromClimbRate(wp_nav._pilot_desired_climb_rate, POSCONTROL_DT_100HZ);

	updateZController();


	// send throttle to attitude controller with angle boost
	setThrottleOut(pos_control.throttle_in, 1, POSCONTROL_THROTTLE_CUTOFF_FREQ);

}


//check if auto mode has been enabled
void checkSticksForAutoWPNav()
{
	float deadband_top = MID_STICK_THROTTLE + THROTTLE_DEADZONE;
	float deadband_bottom = MID_STICK_THROTTLE - THROTTLE_DEADZONE;

	if(rc_in[0] > deadband_bottom && rc_in[0] < deadband_bottom &&
		rc_in[1] > deadband_bottom && rc_in[1] < deadband_bottom &&
		rc_in[2] > deadband_bottom && rc_in[2] < deadband_bottom &&
		rc_in[3] > deadband_bottom && rc_in[3] < deadband_bottom)
	{
		if(wp_nav.count_wp_enable < AUTO_WPNAV_COUNT_THRESHOLD)
			wp_nav.count_wp_enable++;
	}
	else
	{
		if(wp_nav.count_wp_enable > 0)
			wp_nav.count_wp_enable--;
	}
}

// generate new velocities for waypoints if stick is at mid
void getWPNavDesiredVelocity()
{
	Vector2f velocity_xy_des;
	float velocity_z_des;

	velocity_xy_des.x = WPNAV_Kp_POS_XY * (wp_nav.waypoint.x - pos_control.pos_target.x);
	velocity_xy_des.y = WPNAV_Kp_POS_XY * (wp_nav.waypoint.y - pos_control.pos_target.y);
	velocity_z_des = WPNAV_Kp_POS_Z * (wp_nav.waypoint.z - pos_control.pos_target.z);

	float vel_xy = normVec2f(velocity_xy_des);
	if(vel_xy > WPNAV_LOITER_SPEED_MIN && vel_xy > 0)
	{
		velocity_xy_des.x = velocity_xy_des.x * WPNAV_LOITER_SPEED_MIN/vel_xy;
		velocity_xy_des.y = velocity_xy_des.y * WPNAV_LOITER_SPEED_MIN/vel_xy;
	}

	if(abs(velocity_z_des) > WPNAV_WP_SPEED_DOWN && abs(velocity_z_des)>0)
	{
		if(velocity_z_des > 0)
			velocity_z_des = WPNAV_WP_SPEED_DOWN;
		if(velocity_z_des < 0)
			velocity_z_des = -WPNAV_WP_SPEED_DOWN;
	}
}

void getPilotDesiredAcceleration()
{
	int16_t control_pitch_in = constrain_int(rc_in[1], STICK_MIN, STICK_MAX);
	int16_t control_roll_in = constrain_int(rc_in[0], STICK_MIN, STICK_MAX);

	int16_t control_pitch = (control_pitch_in - STICK_MID);
	int16_t control_roll = (control_roll_in - STICK_MID);

	if(abs(control_pitch) < STICK_DEADBAND)
		control_pitch = 0;
	if(abs(control_roll) < STICK_DEADBAND)
		control_roll = 0;

	wp_nav._pilot_accel_fwd_cms = -control_pitch * wp_nav._loiter_accel_cmss / ((STICK_MAX-STICK_MIN)/2);
	wp_nav._pilot_accel_rgt_cms = control_roll * wp_nav._loiter_accel_cmss / ((STICK_MAX-STICK_MIN)/2);
}

void getPilotDesiredXYVelocity()
{
	Vector2f pilot_desired_vel, desired_vel;
	float stick_roll, stick_pitch;

	float deadband_top = MID_STICK_THROTTLE + THROTTLE_DEADZONE;
	float deadband_bottom = MID_STICK_THROTTLE - THROTTLE_DEADZONE;

	//failsafe to detect any unwanted output
	if(isnan(rc_in[0])|| isnan(rc_in[1]))
		return;

	if(isinf(rc_in[0])|| isinf(rc_in[1]))
		return;

	if(rc_in[0] < THROTTLE_MIN || rc_in[0] > THROTTLE_MAX)
		return;

	if(rc_in[1] < THROTTLE_MIN || rc_in[1] > THROTTLE_MAX)
		return;


	stick_roll = constrain_float(rc_in[0],THROTTLE_MIN,THROTTLE_MAX);
	stick_pitch = constrain_float(rc_in[1],THROTTLE_MIN,THROTTLE_MAX);

	if (stick_roll < deadband_bottom)
		pilot_desired_vel.y = wp_nav._pilot_max_xy_speed * (stick_roll-deadband_bottom) / (deadband_bottom-THROTTLE_MIN);
	else
	{
		if (stick_roll > deadband_top)
			pilot_desired_vel.y = wp_nav._pilot_max_xy_speed * (stick_roll-deadband_top) / (THROTTLE_MAX-deadband_top);
		else
			pilot_desired_vel.y = 0.0f;
	}

	if (stick_pitch < deadband_bottom)
		pilot_desired_vel.x = -wp_nav._pilot_max_xy_speed * (stick_pitch-deadband_bottom) / (deadband_bottom-THROTTLE_MIN);
	else
	{
		if (stick_pitch > deadband_top)
			pilot_desired_vel.x = -wp_nav._pilot_max_xy_speed * (stick_pitch-deadband_top) / (THROTTLE_MAX-deadband_top);
		else
			pilot_desired_vel.x = 0.0f;
	}

	float jerk_xy = pos_control.accel_cms * POSCONTROL_JERK_RATIO;

	desired_vel.x = (pilot_desired_vel.x*ahrs.cos_psi - pilot_desired_vel.y*ahrs.sin_psi);
	desired_vel.y = (pilot_desired_vel.x*ahrs.sin_psi + pilot_desired_vel.y*ahrs.cos_psi);

	Vector2f vel_diff;
	vel_diff.x = desired_vel.x - pos_control.vel_desired.x;
	vel_diff.y = desired_vel.y - pos_control.vel_desired.y;

	//TODO REMOVE THIS after ensuring change targets are working fine
//	debug("px is %.2f, py is %.2f; vx %.2f, vy %.2f; posvx %.2f, posvy %.2f; yaw %.2f", pilot_desired_vel.x, pilot_desired_vel.y,
//															desired_vel.x, desired_vel.y,
//															pos_control.vel_desired.x, pos_control.vel_desired.y,
//															ahrs.attitude.z);

	float accel_xy_max = min(pos_control.accel_cms, sqrt(2.0f*fabsf(normVec2f(vel_diff))*jerk_xy));

	pos_control.accel_last_xy_cms += jerk_xy * wp_nav._dt_pilot_inp;
	pos_control.accel_last_xy_cms = min(accel_xy_max, pos_control.accel_last_xy_cms);

	float vel_change_limit = pos_control.accel_last_xy_cms * wp_nav._dt_pilot_inp;
	float vel_diff_norm = normVec2f(vel_diff);
	if(vel_diff_norm > vel_change_limit && vel_change_limit > 0)
	{
		vel_diff.x = vel_diff.x*vel_change_limit/vel_diff_norm;
		vel_diff.y = vel_diff.y*vel_change_limit/vel_diff_norm;
	}

	pos_control.vel_desired.x += vel_diff.x;
	pos_control.vel_desired.y += vel_diff.y;

}

void getPilotDesiredYawRate()
{
	if(isnan(rc_in[3]))
		return;

	if(isinf(rc_in[3]))
		return;

	if(rc_in[3] < THROTTLE_MIN || rc_in[3] > THROTTLE_MAX)
		return;

	int16_t control_yaw_rate = constrain_int(rc_in[3], STICK_MIN, STICK_MAX);
	wp_nav._pilot_desired_yaw_rate = (control_yaw_rate - STICK_MID)*STICK_TO_DEGREEPS;
}

void getPilotClimbRate()
{
	if(isnan(rc_in[2]))
		return;

	if(isinf(rc_in[2]))
		return;

	if(rc_in[2] < THROTTLE_MIN || rc_in[2] > THROTTLE_MAX)
		return;

	float desired_rate;

	float deadband_top = MID_STICK_THROTTLE + THROTTLE_DEADZONE;
	float deadband_bottom = MID_STICK_THROTTLE - THROTTLE_DEADZONE;

	// ensure a reasonable throttle value
	float throttle_control = constrain_float(rc_in[2],THROTTLE_MIN,THROTTLE_MAX);

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

