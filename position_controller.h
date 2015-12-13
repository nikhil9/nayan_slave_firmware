/*
 * position_controller.h
 *
 *  Created on: 05-Oct-2015
 *      Author: atulya
 */

#include "main.h"

#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

// position controller default definitions
#define POSCONTROL_THROTTLE_HOVER               1550.0f  // IGNORE changed default throttle required to maintain hover
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define POSCONTROL_STOPPING_DIST_Z_MAX          200.0f  // max stopping distance vertically
                                                        // should be 1.5 times larger than POSCONTROL_ACCELERATION.
                                                        // max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s	IGNORE added by atulya
#define POSCONTROL_SPEED_DOWN                  -150.0f   // default descent rate in cm/s IGNORE added by atulya
#define POSCONTROL_SPEED_UP                     250.0f   // default climb rate in cm/s IGNORE added by atulya
#define POSCONTROL_VEL_XY_MAX_FROM_POS_ERR      200.0f   // max speed output from pos_to_vel controller when feed forward is used IGNORE added by atulya

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s. IGNORE added by atulya

#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_DT_10HZ                      0.10f   // time difference in seconds for 10hz update rate
#define POSCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define POSCONTROL_DT_100HZ                     0.01f   // time difference in seconds for100hz update rate

#define POSCONTROL_ACTIVE_TIMEOUT_MS            200     // position controller is considered active if it has been called within the past 0.2 seconds

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define POSCONTROL_THROTTLE_CUTOFF_FREQ         2.0f    // low-pass filter on accel error (unit: hz)
#define POSCONTROL_JERK_LIMIT_CMSSS             1700.0f // jerk limit on horizontal acceleration (unit: m/s/s/s)
#define POSCONTROL_ACCEL_FILTER_HZ              2.0f    // low-pass filter on acceleration (unit: hz)
#define POSCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration

///////////Constants defined by atulya ///////////
#define MAX_LEAN_ANGLE							15	//in degrees

#define XY_MODE_POS_ONLY 						0       // position correction only (i.e. no velocity feed-forward)
#define XY_MODE_POS_LIMITED_AND_VEL_FF			1	    // for loiter - rate-limiting the position correction, velocity feed-forward
#define XY_MODE_POS_AND_VEL_FF					2		// for velocity controller - unlimied position correction, velocity feed-forward

/**
 * @brief defines the struct for handling the position controller
 */
typedef struct
{
	struct poscontrol_flags {
	            uint16_t recalc_leash_z     : 1;    // 1 if we should recalculate the z axis leash length
	            uint16_t recalc_leash_xy    : 1;    // 1 if we should recalculate the xy axis leash length
	            uint16_t reset_desired_vel_to_pos   : 1;    // 1 if we should reset the rate_to_accel_xy step
	            uint16_t reset_rate_to_accel_xy     : 1;    // 1 if we should reset the rate_to_accel_xy step
	            uint16_t reset_accel_to_lean_xy     : 1;    // 1 if we should reset the accel to lean angle step
	            uint16_t reset_rate_to_accel_z      : 1;    // 1 if we should reset the rate_to_accel_z step
	            uint16_t reset_accel_to_throttle    : 1;    // 1 if we should reset the accel_to_throttle step of the z-axis controller
	            uint16_t freeze_ff_xy       : 1;    // 1 use to freeze feed forward during step updates
	            uint16_t freeze_ff_z        : 1;    // 1 use to freeze feed forward during step updates
	            uint16_t xy_control_to_pilot : 1;	// IGNORE added by atulya 1->RP control with pilot 0-> RP with HLP
	    } _flags;

	struct poscontrol_limit_flags {
			uint8_t pos_up      : 1;    // 1 if we have hit the vertical position leash limit while going up
			uint8_t pos_down    : 1;    // 1 if we have hit the vertical position leash limit while going down
			uint8_t vel_up      : 1;    // 1 if we have hit the vertical velocity limit going up
			uint8_t vel_down    : 1;    // 1 if we have hit the vertical velocity limit going down
			uint8_t accel_xy    : 1;    // 1 if we have hit the horizontal accel limit
		} _limit;

	Controller_P       _p_pos_z;
	Controller_P       _p_vel_z;
	Controller_PID     _pid_accel_z;
	Controller_P	   _p_pos_xy;
	Controller_PI_2D      _pi_vel_xy;

	// parameters
	float    accel_xy_filt_hz;      // XY acceleration filter cutoff frequency

	// internal variables
	float       dt;                    // time difference (in seconds) between calls from the main program
	float       dt_xy;                 // time difference (in seconds) between update_xy_controller and update_vel_controller_xyz calls
	uint32_t    last_update_xy_ms;     // system time of last update_xy_controller call
	uint32_t    last_update_z_ms;      // system time of last update_z_controller call
	float       throttle_hover;        // estimated throttle required to maintain a level hover
	float       speed_down_cms;        // max descent rate in cm/s
	float       speed_up_cms;          // max climb rate in cm/s
	float       speed_cms;             // max horizontal speed in cm/s
	float       accel_z_cms;           // max vertical acceleration in cm/s/s
	float       accel_last_z_cms;      // max vertical acceleration in cm/s/s
	float       accel_last_xy_cms;     // previous desired acceleration in cms
	float       accel_cms;             // max horizontal acceleration in cm/s/s
	float       leash;                 // horizontal leash length in cm.  target will never be further than this distance from the vehicle
	float       leash_down_z;          // vertical leash down in cm.  target will never be further than this distance below the vehicle
	float       leash_up_z;            // vertical leash up in cm.  target will never be further than this distance above the vehicle

	// output from controller
	float       roll_target;           // desired roll angle in centi-degrees calculated by position controller
	float       pitch_target;          // desired roll pitch in centi-degrees calculated by position controller


	// position controller internal variables
	Vector3f    pos_target;            // target location in cm from home
	Vector3f    pos_error;             // error between desired and actual position in cm
	Vector3f    vel_desired;           // desired velocity in cm/s
	Vector3f    vel_target;            // velocity target in cm/s calculated by pos_to_rate step
	Vector3f    vel_error;             // error between desired and actual acceleration in cm/s
	Vector3f    vel_last;              // previous iterations velocity in cm/s
	Vector3f    accel_target;          // desired acceleration in cm/s/s  // To-Do: are xy actually required?
	Vector3f    accel_error;           // desired acceleration in cm/s/s  // To-Do: are xy actually required?
	Vector3f    accel_feedforward;     // feedforward acceleration in cm/s/s
	float       alt_max;               // max altitude - should be updated from the main code with altitude limit from fence
	float 		alt_min;			   // min altitude - IGNORE added by atulya
	float       distance_to_target;    // distance to position target - for reporting only
	LowPassFilter vel_error_filter;   // low-pass-filter on z-axis velocity error

	float			throttle_in;			// throttle input from the controller(based on z) IGNORE dded by atulya

	int16_t			roll_out;				// final roll output from the controller IGNORE added by atulya
	int16_t			pitch_out;				// final pitch output from the controller IGNORE added by atulya
	int16_t			yaw_rate_out;			// final pitch output from the controller IGNORE added by atulya
	int16_t 		throttle_out;		 	// final throttle output from the controller IGNORE added by atulya

	Vector2f    accel_target_jerk_limited; // acceleration target jerk limited to 100deg/s/s
	LowPassFilter accel_target_filter_x; // acceleration target filter
	LowPassFilter accel_target_filter_y; // acceleration target filter

	LowPassFilter throttle_in_filter;		//IGNORE added by atulya

}Position_Controller;

void initializePosController(void);

void resetController(void);

void setAttitude(float roll, float pitch, float yaw_rate);

void setAltTargetfromClimbRate(float climb_rate_cms, float dt);

void setThrottleOut(float throttle_in, uint8_t apply_angle_boost, float filt_hz);

void updateXYController(int mode, int use_althold_lean_angle);

void updateZController(void);

#endif /* POSITION_CONTROLLER_H_ */
