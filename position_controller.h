/*
 * position_controller.h
 *
 *  Created on: 05-Oct-2015
 *      Author: atulya
 */

#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "main.h"

/**
 * @brief updates the controller on the basis of PI on the error
 */

typedef struct
{
	float kP;
}Controller_P;

typedef struct
{
	float kP;
	float kI;
	float Imax;
	float filt_hz;
	uint8_t reset_filter;

	float dt;
	float input;
	float integrator;
	float filt_alpha;
	float result;
}Controller_PI;

typedef struct
{
	float kP;
	float kI;
	float kD;
	float Imax;
	float filt_hz;
	uint8_t reset_filter;

	float dt;
	float input;
	float integrator;
	float derivative;
	float filt_alpha;
	float result;
}Controller_PID;



/**
 * @brief defines the struct for handling the position controller
 */
typedef struct
{
	float       p_pos_z;
	float       p_vel_z;
	Controller_PID     _pid_accel_z;
	float	    _p_pos_xy;
	Controller_PI   _pi_vel_xy;

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
	float       distance_to_target;    // distance to position target - for reporting only
//	LowPassFilterFloat vel_error_filter;   // low-pass-filter on z-axis velocity error

	Vector2f    accel_target_jerk_limited; // acceleration target jerk limited to 100deg/s/s
//	LowPassFilterVector2f accel_target_filter; // acceleration target filter

	Vector3f pos_desired;

}Position_Controller;

extern Position_Controller pos_control;

/**
 * @brief sets the desired acceleration by taking feedback from the pilot input TODO
 */
void setPilotDesiredAcceleration(void);

/**
 * @brief sets the desired velocity from the pilot desired acceleration TODO
 */
void setPilotDesiredvelocity(void);

/**
 * @brief updates the position controller on the basis of feedback from INAV and using the desired position and velocity from above TODO
 */
void updateXYController(void);

/**
 * @brief executes the loiter code TODO
 */
void loiter_run(void);

////////////////CONTROLLER LIBRARIES/////////////////////

/////////////------------PI-----------////////////

// reset_I - reset the integrator TODO
void resetPI_I(Controller_PI *pi);

// reset_filter - input filter will be reset to the next value provided to set_input() TODO
void resetPI_filter(Controller_PI *pi);

// TODO
void initializePI(Controller_PI *pi);

// TODO
void setPIInput(Controller_PI *pi, float input, float dt);

// TODO
void updatePIOutput(Controller_PI *pi);

////////////////////////PID///////////////////////
// resetPID_I - reset the integrator TODO
void resetPID_I(Controller_PID *pid);

// reset_filter - input filter will be reset to the next value provided to set_input() TODO
void resetPID_filter(Controller_PID *pid);

//initialize the PID controller TODO
void intializePID(Controller_PID *pid);

/**
 * @brief updates the input and dt of the given PID controller TODO
 */
void setPIDInputFilterAll(Controller_PID *pid, float input, float dt);

/**
 * @brief gives the result of the PID controller acted on the input  TODO
 */
void updatePIDOutput(Controller_PID *pid);
//////////////////////////////////////////////////
#endif /* POSITION_CONTROLLER_H_ */
