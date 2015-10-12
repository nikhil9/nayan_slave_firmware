/*
 * wp_nav.h
 *
 *  Created on: 10-Oct-2015
 *      Author: atulya
 */

#include "main.h"

#ifndef WP_NAV_H_
#define WP_NAV_H_

#define STICK_TO_DEGREEPSS			1.0
#define MID_STICK					2048
#define THROTTLE_DEADZONE			300
#define THROTTLE_MIN				1000
#define THROTTLE_MAX				4000

typedef struct ap_waypoint_nav
{
	float    _loiter_speed_cms;      // maximum horizontal speed in cm/s while in loiter
    float    _loiter_jerk_max_cmsss; // maximum jerk in cm/s/s/s while in loiter
    float    _loiter_accel_cmss;     // loiter's max acceleration in cm/s/s
    float    _loiter_accel_min_cmss; // loiter's min acceleration in cm/s/s
    float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    float    _wp_speed_up_cms;       // climb speed target in cm/s
    float    _wp_speed_down_cms;     // descent speed target in cm/s
    float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    float    _wp_accel_cms;          // horizontal acceleration in cm/s/s during missions
    float    _wp_accel_z_cms;        // vertical acceleration in cm/s/s during missions

    struct wpnav_flags {
            uint8_t reached_destination     : 1;    // true if we have reached the destination
            uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
            uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
            uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
            uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
        } _flags;

    // loiter controller internal variables
    uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    int16_t     _pilot_accel_fwd_cms; 	// pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_accel_rgt_cms;   // pilot's desired acceleration right (body-frame)
    int16_t     _pilot_desired_yaw_rate;// pilot's desired yaw rate		IGNORE added by atulya
    int16_t     _pilot_desired_climb_rate;// pilot's desired climb rate in cms	IGNORE added by atulya
    int16_t		_pilot_max_z_velocity;	//max z velocity from the pilot	IGNORE added by atulya
    uint32_t	_last_pilot_update_ms;	//timestamp of the last update performed by the pilot inputs to the loiter code IGNORE
    float		_dt_pilot_inp;			//time duration in which to check the pilot input
    Vector2f    _loiter_desired_accel;  // slewed pilot's desired acceleration in lat/lon frame

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
    float       _slow_down_dist;        // vehicle should begin to slow down once it is within this distance from the destination

    // spline variables
    float       _spline_time;           // current spline time between origin and destination
    float       _spline_time_scale;     // current spline time between origin and destination
    Vector3f    _spline_origin_vel;     // the target velocity vector at the origin of the spline segment
    Vector3f    _spline_destination_vel;// the target velocity vector at the destination point of the spline segment
    Vector3f    _hermite_spline_solution[4]; // array describing spline path between origin and destination
    float       _spline_vel_scaler;	    //
    float       _yaw;                   // heading according to yaw
}WP_Nav;

void updateLoiter(void);

void loiter_run(void);					//TODO

void getPilotDesiredAcceleration(void);
void getPilotDesiredYawRate(void);
void getPilotClimbRate(void);

void updateAltHold(void);

#endif /* WP_NAV_H_ */
