/*
 * position_controller.c
 *
 *  Created on: 05-Oct-2015
 *      Author: atulya
 */
#include "main.h"

void initializePosController()
{
	pos_control._p_pos_z.kP = ALT_HOLD_P;
	pos_control._p_vel_z.kP = VEL_Z_P;
	initializePID(&pos_control._pid_accel_z, ACCEL_Z_P, ACCEL_Z_I, ACCEL_Z_D, ACCEL_Z_IMAX, ACCEL_Z_FILT_HZ);
	pos_control._p_pos_xy.kP = POS_XY_P;
	initializePI(&pos_control._pi_vel_xy, VEL_XY_P, VEL_XY_I, VEL_XY_IMAX, VEL_XY_FILT_HZ);

	pos_control.dt = POSCONTROL_DT_100HZ;
	pos_control.dt_xy = POSCONTROL_DT_50HZ;
	pos_control.last_update_xy_ms = 0;
	pos_control.last_update_z_ms = 0;
	pos_control.throttle_hover = POSCONTROL_THROTTLE_HOVER;
	pos_control.speed_down_cms = POSCONTROL_SPEED_DOWN;
	pos_control.speed_up_cms = POSCONTROL_SPEED_UP;
	pos_control.speed_cms = POSCONTROL_SPEED;
	pos_control.accel_z_cms = POSCONTROL_ACCEL_Z;
	pos_control.accel_last_z_cms = 0.0f;
	pos_control.accel_cms = POSCONTROL_ACCEL_XY;
	pos_control.leash = POSCONTROL_LEASH_LENGTH_MIN;
	pos_control.leash_down_z = POSCONTROL_LEASH_LENGTH_MIN;
	pos_control.leash_up_z = POSCONTROL_LEASH_LENGTH_MIN;
	pos_control.roll_target = 0.0f;
	pos_control.pitch_target = 0.0f;
	pos_control.accel_xy_filt_hz = POSCONTROL_ACCEL_FILTER_HZ;
	pos_control.throttle_in_filter.cutoff_freq = POSCONTROL_THROTTLE_CUTOFF_FREQ;
	pos_control.throttle_in_filter.output = MID_STICK_THROTTLE;

	initializeVector3fToZero(&pos_control.pos_target);
	initializeVector3fToZero(&pos_control.pos_error);
	initializeVector3fToZero(&pos_control.vel_desired);
	initializeVector3fToZero(&pos_control.vel_target);
	initializeVector3fToZero(&pos_control.vel_error);
	initializeVector3fToZero(&pos_control.vel_last);
	initializeVector3fToZero(&pos_control.accel_target);
	initializeVector3fToZero(&pos_control.accel_error);
	initializeVector3fToZero(&pos_control.accel_feedforward);

	pos_control.throttle_out = pos_control.throttle_hover;
	pos_control.pitch_out = STICK_MID;
	pos_control.roll_out = STICK_MID;
	pos_control.yaw_rate_out = STICK_MID;

	ic_rc_or_data.ic_rc.rc1 = pos_control.roll_out;
	ic_rc_or_data.ic_rc.rc2 = pos_control.pitch_out;
	ic_rc_or_data.ic_rc.rc3 = pos_control.throttle_out;
	ic_rc_or_data.ic_rc.rc4 = pos_control.yaw_rate_out;

	pos_control.alt_max = POSTARGET_MAX_ALTITUDE;
	pos_control.alt_min = POSTARGET_MIN_ALTITUDE;
	pos_control.distance_to_target = 0.0f;
	pos_control.vel_error_filter.cutoff_freq = POSCONTROL_VEL_ERROR_CUTOFF_FREQ;
	pos_control.accel_target_jerk_limited.x = 0.0f;
	pos_control.accel_target_jerk_limited.y = 0.0f;
	pos_control.accel_target_filter_x.cutoff_freq = POSCONTROL_ACCEL_FILTER_HZ;
	pos_control.accel_target_filter_y.cutoff_freq = POSCONTROL_ACCEL_FILTER_HZ;

	// initialise flags
	pos_control._flags.recalc_leash_z = 1;
	pos_control._flags.recalc_leash_xy = 1;
	pos_control._flags.reset_desired_vel_to_pos = 1;
	pos_control._flags.reset_rate_to_accel_xy = 1;
	pos_control._flags.reset_accel_to_lean_xy = 1;
	pos_control._flags.reset_rate_to_accel_z = 1;
	pos_control._flags.reset_accel_to_throttle = 1;
	pos_control._flags.freeze_ff_xy = 1;
	pos_control._flags.freeze_ff_z = 1;
	pos_control._limit.pos_up = 1;
	pos_control._limit.pos_down = 1;
	pos_control._limit.vel_up = 1;
	pos_control._limit.vel_down = 1;
	pos_control._limit.accel_xy = 1;

}

void resetController()
{
	resetPI_I(&pos_control._pi_vel_xy);
	resetPID_I(&pos_control._pid_accel_z);

	pos_control.pos_target.z = inav.position.z;
	pos_control.vel_desired.z = 0;

	wp_nav._loiter_desired_accel.x = 0;
	wp_nav._loiter_desired_accel.y = 0;
	pos_control.vel_desired.x = 0;
	pos_control.vel_desired.y = 0;
	pos_control.pos_target.x = inav.position.x;
	pos_control.pos_target.y = inav.position.y;
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
static float calcLeashLength(float speed_cms, float accel_cms, float kP)
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}


static void desiredVelToPos(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    if (pos_control._flags.reset_desired_vel_to_pos) {
        pos_control._flags.reset_desired_vel_to_pos = 0;
    } else
    {
        pos_control.pos_target.x += pos_control.vel_desired.x * nav_dt;
        pos_control.pos_target.y += pos_control.vel_desired.y * nav_dt;
    }
}

static float sqrtController(float error, float p, float second_ord_lim)
{
	if(second_ord_lim == 0 || p==0)
		return error*p;

	float linear_dist = second_ord_lim/(p*p);

	if (error > linear_dist)
	{
		return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
	}
	else
	{
		if (error < -linear_dist)
			return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
		else
			return error*p;
	}
}

static void posToRateXY(int mode, float dt)
{
    Vector3f curr_pos = inav.position;
    float linear_distance;      // the distance we swap between linear and sqrt velocity response
    float kP = 1.0* pos_control._p_pos_xy.kP; // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        pos_control.vel_target.x = 0.0f;
        pos_control.vel_target.y = 0.0f;
    }
    else
    {
        // calculate distance error
        pos_control.pos_error.x = pos_control.pos_target.x - curr_pos.x;
        pos_control.pos_error.y = pos_control.pos_target.y - curr_pos.y;

        // constrain target position to within reasonable distance of current location
        pos_control.distance_to_target = pythagorous2(pos_control.pos_error.x, pos_control.pos_error.y);
        if (pos_control.distance_to_target > pos_control.leash && pos_control.distance_to_target > 0.0f)
        {
            pos_control.pos_target.x = curr_pos.x + pos_control.leash * pos_control.pos_error.x/pos_control.distance_to_target;
            pos_control.pos_target.y = curr_pos.y + pos_control.leash * pos_control.pos_error.y/pos_control.distance_to_target;
            // re-calculate distance error
            pos_control.pos_error.x = pos_control.pos_target.x - curr_pos.x;
            pos_control.pos_error.y = pos_control.pos_target.y - curr_pos.y;
            pos_control.distance_to_target = pos_control.leash;
        }

        // calculate the distance at which we swap between linear and sqrt velocity response
        linear_distance = pos_control.accel_cms/(2.0f*kP*kP);

        if (pos_control.distance_to_target > 2.0f*linear_distance) {
            // velocity response grows with the square root of the distance
            float vel_sqrt = sqrt(2.0f*pos_control.accel_cms*(pos_control.distance_to_target-linear_distance));
            pos_control.vel_target.x = vel_sqrt * pos_control.pos_error.x/pos_control.distance_to_target;
            pos_control.vel_target.y = vel_sqrt * pos_control.pos_error.y/pos_control.distance_to_target;
        }else{
            // velocity response grows linearly with the distance
            pos_control.vel_target.x = pos_control._p_pos_xy.kP * pos_control.pos_error.x;
            pos_control.vel_target.y = pos_control._p_pos_xy.kP * pos_control.pos_error.y;
        }

        if (mode == XY_MODE_POS_LIMITED_AND_VEL_FF) {
            // this mode is for loiter - rate-limiting the position correction
            // allows the pilot to always override the position correction in
            // the event of a disturbance

            // scale velocity within limit
            float vel_total = pythagorous2(pos_control.vel_target.x, pos_control.vel_target.y);
            if (vel_total > POSCONTROL_VEL_XY_MAX_FROM_POS_ERR) {
                pos_control.vel_target.x = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * pos_control.vel_target.x/vel_total;
                pos_control.vel_target.y = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * pos_control.vel_target.y/vel_total;
            }

            // add velocity feed-forward
            pos_control.vel_target.x += pos_control.vel_desired.x;
            pos_control.vel_target.y += pos_control.vel_desired.y;
        } else {
            if (mode == XY_MODE_POS_AND_VEL_FF) {
                // add velocity feed-forward
                pos_control.vel_target.x += pos_control.vel_desired.x;
                pos_control.vel_target.y += pos_control.vel_desired.y;
            }

            // scale velocity within speed limit
            float vel_total = pythagorous2(pos_control.vel_target.x, pos_control.vel_target.y);
            if (vel_total > pos_control.speed_cms) {
                pos_control.vel_target.x = pos_control.speed_cms * pos_control.vel_target.x/vel_total;
                pos_control.vel_target.y = pos_control.speed_cms * pos_control.vel_target.y/vel_total;
            }
        }
    }
}
// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
static void rateToAccelXY(float dt)
{
    const Vector3f vel_curr = inav.velocity;  // current velocity in cm/s
    Vector2f vel_xy_p, vel_xy_i;

    // reset last velocity target to current target
    if (pos_control._flags.reset_rate_to_accel_xy) {
        pos_control.vel_last.x = pos_control.vel_target.x;
        pos_control.vel_last.y = pos_control.vel_target.y;
        pos_control._flags.reset_rate_to_accel_xy = 0;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
    	if (!pos_control._flags.freeze_ff_xy) {
    		pos_control.accel_feedforward.x = (pos_control.vel_target.x - pos_control.vel_last.x)/dt;
    		pos_control.accel_feedforward.y = (pos_control.vel_target.y - pos_control.vel_last.y)/dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		pos_control._flags.freeze_ff_xy = 0;
    	}
    } else {
    	pos_control.accel_feedforward.x = 0.0f;
    	pos_control.accel_feedforward.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    pos_control.vel_last.x = pos_control.vel_target.x;
    pos_control.vel_last.y = pos_control.vel_target.y;

    // calculate velocity error
    pos_control.vel_error.x = pos_control.vel_target.x - vel_curr.x;
    pos_control.vel_error.y = pos_control.vel_target.y - vel_curr.y;

    // call pi controller
    Vector2f velxy_error;
    velxy_error.x = pos_control.vel_error.x;
    velxy_error.y = pos_control.vel_error.y;
    setPIInput(&pos_control._pi_vel_xy, velxy_error, dt);
//    pos_control._pi_vel_xy.set_input(pos_control.vel_error);

    // get p
    vel_xy_p = getPI_P(&pos_control._pi_vel_xy);

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    if (!pos_control._limit.accel_xy )
    	vel_xy_i = getPI_I(&pos_control._pi_vel_xy);
    else {
        vel_xy_i = getPI_I_shrink(&pos_control._pi_vel_xy);
    }

    // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    pos_control.accel_target.x = pos_control.accel_feedforward.x + (vel_xy_p.x + vel_xy_i.x) * 1.0;
    pos_control.accel_target.y = pos_control.accel_feedforward.y + (vel_xy_p.y + vel_xy_i.y) * 1.0;
}

static void accelToLeanAngles(float dt, int use_althold_lean_angle)
{
    float accel_total;                          // total acceleration in cm/s/s
    float accel_right, accel_forward;
    float lean_angle_max = MAX_LEAN_ANGLE;
    float accel_max = POSCONTROL_ACCEL_XY;

    // limit acceleration if necessary
    if (use_althold_lean_angle) {
        accel_max = min(accel_max, GRAVITY_MSS * 100.0f * sinf(DEG_TO_RAD*constrain_float(MAX_LEAN_ANGLE,10,45)));
    }

    debug("accel_max is %f", accel_max);

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = pythagorous2(pos_control.accel_target.x, pos_control.accel_target.y);
    if (accel_total > accel_max && accel_total > 0.0f) {
        pos_control.accel_target.x = accel_max * pos_control.accel_target.x/accel_total;
        pos_control.accel_target.y = accel_max * pos_control.accel_target.y/accel_total;
        pos_control._limit.accel_xy = 1;     // unused
    } else {
        // reset accel limit flag
        pos_control._limit.accel_xy = 0;
    }

    // reset accel to current desired acceleration
    if (pos_control._flags.reset_accel_to_lean_xy) {
        pos_control.accel_target_jerk_limited.x = pos_control.accel_target.x;
        pos_control.accel_target_jerk_limited.y = pos_control.accel_target.y;

        resetLPF(&pos_control.accel_target_filter_x, pos_control.accel_target.x);
        resetLPF(&pos_control.accel_target_filter_y, pos_control.accel_target.y);
        pos_control._flags.reset_accel_to_lean_xy = 0;
    }

    // apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
    float max_delta_accel = dt * POSCONTROL_JERK_LIMIT_CMSSS;

    Vector2f accel_in;
    accel_in.x = pos_control.accel_target.x;
    accel_in.y = pos_control.accel_target.y;
    Vector2f accel_change;
    accel_change.x = accel_in.x-pos_control.accel_target_jerk_limited.x;
    accel_change.y = accel_in.y-pos_control.accel_target_jerk_limited.y;
    float accel_change_length = pythagorous2(accel_change.x, accel_change.y);

    if(accel_change_length > max_delta_accel)
    {
        accel_change.x *= max_delta_accel/accel_change_length;
        accel_change.y *= max_delta_accel/accel_change_length;
    }
    pos_control.accel_target_jerk_limited.x += accel_change.x;
    pos_control.accel_target_jerk_limited.y += accel_change.y;

    // lowpass filter on NE accel
    pos_control.accel_target_filter_x.cutoff_freq = pos_control.accel_xy_filt_hz;
    pos_control.accel_target_filter_y.cutoff_freq = pos_control.accel_xy_filt_hz;

    Vector2f accel_target_filtered;
    accel_target_filtered.x = applyLPF(&pos_control.accel_target_filter_x, pos_control.accel_target_jerk_limited.x, dt);
    accel_target_filtered.y = applyLPF(&pos_control.accel_target_filter_y, pos_control.accel_target_jerk_limited.y, dt);

    // rotate accelerations into body forward-right frame
    accel_forward = accel_target_filtered.x*ahrs.cos_psi + accel_target_filtered.y*ahrs.sin_psi;
    accel_right = -accel_target_filtered.x*ahrs.sin_psi + accel_target_filtered.y*ahrs.cos_psi;

    // update angle targets that will be passed to stabilize controller
    pos_control.pitch_target = constrain_float(atan(-accel_forward/(GRAVITY_MSS * 100))*(RAD_TO_DEG),-lean_angle_max, lean_angle_max);
    float cos_pitch_target = cosf(pos_control.pitch_target*DEG_TO_RAD);
    pos_control.roll_target = constrain_float(atan(accel_right*cos_pitch_target/(GRAVITY_MSS * 100))*(RAD_TO_DEG), -lean_angle_max, lean_angle_max);
}


void updateXYController(int mode, int use_althold_lean_angle)
{
	// compute dt
	uint32_t now = millis();
	float dt = (now - pos_control.last_update_xy_ms) / 1000.0f;
	pos_control.last_update_xy_ms = now;

	// sanity check dt - expect to be called faster than ~5hz
	if (dt > POSCONTROL_ACTIVE_TIMEOUT_MS*1.0e-3f) {
		dt = 0.0f;
	}

	// check if xy leash needs to be recalculated
	if(pos_control._flags.recalc_leash_xy)
	{
		pos_control.leash = 	calcLeashLength(pos_control.speed_cms, pos_control.accel_cms, pos_control._p_pos_xy.kP);
		pos_control._flags.recalc_leash_xy = 0;
	}

	// translate any adjustments from pilot to loiter target
	desiredVelToPos(dt);

	// run position controller's position error to desired velocity step
	posToRateXY(mode, dt);

	// run position controller's velocity to acceleration step
	rateToAccelXY(dt);

	// run position controller's acceleration to lean angle step
	accelToLeanAngles(dt, use_althold_lean_angle);

}

static void accelToThrottle(float accel_target_z)
{

	float z_accel_meas;         // actual acceleration
	float p,i,d;              // used to capture pid values for logging

	z_accel_meas = -(ahrs.accel_ef.z*100 + GRAVITY_CMSS);

	// reset target altitude if this controller has just been engaged
	if (pos_control._flags.reset_accel_to_throttle)
	{
		// Reset Filter
		pos_control.accel_error.z = 0;
		pos_control._flags.reset_accel_to_throttle = 0;
	}
	else {
		// calculate accel error
		pos_control.accel_error.z = accel_target_z - z_accel_meas;
	}

	// set input to PID
	setPIDInput_FilterD(&pos_control._pid_accel_z, pos_control.accel_error.z);

	// separately calculate p, i, d values for logging
	p = getPID_P(&pos_control._pid_accel_z);

	// get i term
	i = getPID_I(&pos_control._pid_accel_z);

	//TODO add a check to ensure that when motors have reached throttle limit then do not integrate
//	// update i term as long as we haven't breached the limits or the I term will certainly reduce
//	// To-Do: should this be replaced with limits check from attitude_controller?
//	    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
//	        i = _pid_accel_z.get_i();
//	    }

	// get d term
	d = getPID_D(&pos_control._pid_accel_z);

	pos_control.throttle_in = p+i+d+pos_control.throttle_hover;

//	    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);			//used in wp_nav.c loiter_run

}
static void rateToAccelZ(void)
{
	Vector3f curr_vel = inav.velocity;
	float p;                                // used to capture pid values for logging

	// check speed limits
	// To-Do: check these speed limits here or in the pos->rate controller
	pos_control._limit.vel_up = 0;
	pos_control._limit.vel_down = 0;
	if (pos_control.vel_target.z < pos_control.speed_down_cms)
	{
		pos_control.vel_target.z = pos_control.speed_down_cms;
		pos_control._limit.vel_down = 1;
	}
	if (pos_control.vel_target.z > pos_control.speed_up_cms)
	{
		pos_control.vel_target.z = pos_control.speed_up_cms;
		pos_control._limit.vel_up = 1;
	}

	// reset last velocity target to current target
	if (pos_control._flags.reset_rate_to_accel_z)
		pos_control.vel_last.z = pos_control.vel_target.z;


	// feed forward desired acceleration calculation
	if (pos_control.dt > 0.0f)
	{
		if (!pos_control._flags.freeze_ff_z)
		{
			pos_control.accel_feedforward.z = (pos_control.vel_target.z - pos_control.vel_last.z)/pos_control.dt;
		}
		else
		{
			// stop the feed forward being calculated during a known discontinuity
			pos_control._flags.freeze_ff_z = 0;
		}
	}
	else
	{
		pos_control.accel_feedforward.z = 0.0f;
	}

	// store this iteration's velocities for the next iteration
	pos_control.vel_last.z = pos_control.vel_target.z;

	// reset velocity error and filter if this controller has just been engaged
	if (pos_control._flags.reset_rate_to_accel_z)
	{
		// Reset Filter
		pos_control.vel_error.z = 0;
		resetLPF(&pos_control.vel_error_filter, 0);
		pos_control._flags.reset_rate_to_accel_z = 0;
	}
	else
	{
		// calculate rate error and filter with cut off frequency of 2 Hz
		pos_control.vel_error.z = applyLPF(&pos_control.vel_error_filter, pos_control.vel_target.z - curr_vel.z, pos_control.dt);
//		debug("applying lpf with freq%f: old val:%f; new_val%f, with dt:%f GIVES %f",pos_control.vel_error_filter.cutoff_freq,
//				pos_control.vel_error_filter.output, pos_control.vel_target.z - curr_vel.z, pos_control.dt, pos_control.vel_error.z);
	    }

	    // calculate p
	    p = pos_control._p_vel_z.kP * pos_control.vel_error.z;

	    // consolidate and constrain target acceleration
	    pos_control.accel_target.z = pos_control.accel_feedforward.z + p;

	    // set target for accel based throttle controller
	    accelToThrottle(pos_control.accel_target.z);

}

static void posToRateZ(void)
{
	float curr_alt = inav.position.z;

	// clear position limit flags
	pos_control._limit.pos_up = 0;
	pos_control._limit.pos_down = 0;

	// calculate altitude error
	pos_control.pos_error.z = pos_control.pos_target.z - curr_alt;

	// do not let target altitude get too far from current altitude
	if (pos_control.pos_error.z > pos_control.leash_up_z)
	{
		pos_control.pos_target.z = curr_alt + pos_control.leash_up_z;
		pos_control.pos_error.z = pos_control.leash_up_z;
		pos_control._limit.pos_up = 1;
	}
	if (pos_control.pos_error.z < -pos_control.leash_down_z)
	{
		pos_control.pos_target.z = curr_alt - pos_control.leash_down_z;
		pos_control.pos_error.z = -pos_control.leash_down_z;
		pos_control._limit.pos_down = 1;
	}

	// calculate pos_control.vel_target.z using from pos_control.pos_error.z using sqrt controller
	pos_control.vel_target.z = sqrtController(pos_control.pos_error.z, pos_control._p_pos_z.kP, pos_control.accel_z_cms);

	// add feed forward component
	pos_control.vel_target.z += pos_control.vel_desired.z;

	// call rate based throttle controller which will update accel based throttle controller targets
	rateToAccelZ();

}

void setAltTargetfromClimbRate(float climb_rate_cms, float dt)
{
    // jerk_z is calculated to reach full acceleration in 1000ms.
    float jerk_z = pos_control.accel_z_cms * POSCONTROL_JERK_RATIO;

    float accel_z_max = min(pos_control.accel_z_cms, sqrt(2.0f*fabsf(pos_control.vel_desired.z - climb_rate_cms)*jerk_z));

    pos_control.accel_last_z_cms += jerk_z * dt;
    pos_control.accel_last_z_cms = min(accel_z_max, pos_control.accel_last_z_cms);

    float vel_change_limit = pos_control.accel_last_z_cms * dt;
    pos_control.vel_desired.z = constrain_float(climb_rate_cms, pos_control.vel_desired.z-vel_change_limit, pos_control.vel_desired.z+vel_change_limit);

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    //IGNORE this check for now
//    if ((pos_control.vel_desired.z<0 && (!_motors.limit.throttle_lower || force_descend)) || (_vel_desired.z>0 && !_motors.limit.throttle_upper && !_limit.pos_up))
	pos_control.pos_target.z += pos_control.vel_desired.z * dt;

    // do not let target alt get above limit
    if (pos_control.alt_max > 0 && pos_control.pos_target.z > pos_control.alt_max) {
        pos_control.pos_target.z = pos_control.alt_max;
        pos_control._limit.pos_up = 1;
        // decelerate feed forward to zero
        if(pos_control.vel_desired.z > 0)
        	pos_control.vel_desired.z = constrain_float(0.0f, pos_control.vel_desired.z-vel_change_limit, pos_control.vel_desired.z+vel_change_limit);

    }
    if (pos_control.pos_target.z < pos_control.alt_min) {
		pos_control.pos_target.z = pos_control.alt_min;
		pos_control._limit.pos_down = 1;
		// decelerate feed forward to zero
		if(pos_control.vel_desired.z < 0)
			pos_control.vel_desired.z = constrain_float(0.0f, pos_control.vel_desired.z-vel_change_limit, pos_control.vel_desired.z+vel_change_limit);
	}
    //TODO this velocity update will fail if quad has gone out of the alt max and min limits and the sensor saturates
}
void updateZController()
{
    // check time since last cast
    uint32_t now = millis();
    if (now - pos_control.last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS)
    {
        pos_control._flags.reset_rate_to_accel_z = 1;
        pos_control._flags.reset_accel_to_throttle = 1;
    }
    pos_control.last_update_z_ms = now;

    // check if leash lengths need to be recalculated
    if(pos_control._flags.recalc_leash_z)
    {
    	pos_control.leash_up_z = calcLeashLength(pos_control.speed_up_cms, pos_control.accel_z_cms, pos_control._p_pos_z.kP);
    	pos_control.leash_down_z = calcLeashLength(-pos_control.speed_down_cms, pos_control.accel_z_cms, pos_control._p_pos_z.kP);
    	pos_control._flags.recalc_leash_z = 0;
    }

    //call position controller
    posToRateZ();
}
void setAttitude(float roll, float pitch, float yaw_rate)
{
	int16_t roll_out = STICK_MID + DEGREE_TO_STICK*roll;
	int16_t pitch_out = STICK_MID + DEGREE_TO_STICK*pitch;
	int16_t yaw_rate_out = STICK_MID + DEGREEPS_TO_STICK*yaw_rate;

	pos_control.roll_out = constrain_int(roll_out, RP_OUTPUT_MIN, RP_OUTPUT_MAX);
	pos_control.pitch_out = constrain_int(pitch_out, RP_OUTPUT_MIN, RP_OUTPUT_MAX);
	pos_control.yaw_rate_out = constrain_int(yaw_rate_out, YAW_OUTPUT_MIN, YAW_OUTPUT_MAX);

	ic_rc_or_data.ic_rc.rc1 = pos_control.roll_out;
	ic_rc_or_data.ic_rc.rc2 = pos_control.pitch_out;
	ic_rc_or_data.ic_rc.rc4 = pos_control.yaw_rate_out;

	//USE THIS IF YOU WANT TO DIRECTLY MAP RC INPUTS TO OVERRIDES
//	ic_rc_or_data.ic_rc.rc1 = rc_in[0];
//	ic_rc_or_data.ic_rc.rc2 = rc_in[1];
//	ic_rc_or_data.ic_rc.rc4 = rc_in[3];
}

void setThrottleOut(float throttle_in, uint8_t apply_angle_boost, float filt_hz)
{
	float throttle_out;
	//TODO check filt_hz and angle_boost
	pos_control.throttle_in_filter.cutoff_freq = filt_hz;
//	ang_vel[0] = throttle_in;
	throttle_in = applyLPF(&pos_control.throttle_in_filter, throttle_in, POSCONTROL_DT_100HZ);
	ang_vel[1] = throttle_in;

	float cos_tilt = ahrs.cos_theta * ahrs.cos_phi;
	float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

	if(apply_angle_boost == 1)
	{
		throttle_out = (throttle_in - (THROTTLE_MIN+130))*boost_factor + (THROTTLE_MIN+130);		//throttle min was 130/1000 from nayan llp parameters
	}
	else
		throttle_out = throttle_in;

//	ang_vel[2] = throttle_out;

	pos_control.throttle_out = constrain_int(throttle_out, THROTTLE_OUTPUT_MIN, THROTTLE_OUTPUT_MAX);
	ic_rc_or_data.ic_rc.rc3 = pos_control.throttle_out;
//	debug("sending out throttle %d", pos_control.throttle_out);

	//USE THIS IF YOU WANT TO DIRECTLY MAP RC INPUTS TO OVERRIDES
//	ic_rc_or_data.ic_rc.rc3 = rc_in[2];
}
