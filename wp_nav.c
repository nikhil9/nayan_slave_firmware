/*
 * wp_nav.c
 *
 *  Created on: 10-Oct-2015
 *      Author: atulya
 */

#include "main.h"

void loiter_run()
{
	setPilotDesiredAcceleration();

	updateLoiter();

	//TODO set the roll pitch yaw angles for the lower level controller
	setAttitude();

	//getPilotDesired climb rate
	setClimbRate();

	//set target altitude based on the desired climb rate
//	setAltTargetfromClimbRate();

	updateZController();
}

void setPilotDesiredAcceleration()
{
	//TODO use the variable rc_in to send inputs in any given range
}

void setClimbRate()
{
	//TODO set the climb rate assuming that pilot input and sonar are available
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
	des_accel_diff.y = (desired_accel.x - wp_nav._loiter_desired_accel.y);

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
	float dt = millis() - pos_control.last_update_xy_ms;

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
