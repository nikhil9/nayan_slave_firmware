/*
 * wp_nav.h
 *
 *  Created on: 10-Oct-2015
 *      Author: atulya
 */

#include "main.h"

#ifndef WP_NAV_H_
#define WP_NAV_H_

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

    // loiter controller internal variables
    uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    int16_t     _pilot_accel_fwd_cms; 	// pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_accel_rgt_cms;   // pilot's desired acceleration right (body-frame)
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

void updateLoiter(void);				//TODO

void loiter_run(void);					//TODO

void setPilotDesiredAcceleration(void);	//TODO

void setClimbRate(void);				//TODO

#endif /* WP_NAV_H_ */
