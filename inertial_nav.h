/*
 * inertial_nav.h
 *
 *  Created on: 29-Sep-2015
 *      Author: atulya
 */

/**
 * @date 29-Sep-2015
 * @author Atulya Shivam Shree
 * @file inertial_nav.h
 * @brief This file implements the basic inertial navigation system using a complementary filter
 */

#include "main.h"

#ifndef INERTIAL_NAV_H_
#define INERTIAL_NAV_H_

#define GRAVITY_MSS 					9.80665f
#define GRAVITY_CMSS 					100*GRAVITY_MSS
#define LATLON_TO_CM 1.113195f

#define INERTIAL_NAV_DELTAT_MAX 		0.1
#define AP_INTERTIALNAV_TC_XY   		2.5f // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    		5.0f // default time constant for complementary filter's Z axis

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   10
#define AP_HISTORIC_XY_SIZE							5
#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
#define AP_INTERTIALNAV_GPS_TIMEOUT_MS              300     // timeout after which position error from GPS will fall to zero

#define AP_HISTORIC_Z_SIZE							15		// assuming a 150 ms delay for the ultrasonic data if the AHRS is called at 100Hz
#define GPS_RADIUS_CM								400

/**
 * @brief implements basic variables required for inertial navigation
 */
typedef struct
{
	Vector3f position_base; 		/**< base position as calculated by the inertial sensors only #position_base. */
	Vector3f position_correction; 	/**< position_correction + position_base = position #position_correction. */
	Vector3f velocity; 				/**< velocity estimate as calculated by the inertial sensors only #velocity. */
	Vector3f position_error; 		/**< position error = (position - (position_base + position_correction)) #position_error. */
	Vector3f position; 				/**< position of the vehicle as calculated by the inertial navigation system #position. */
	Vector3f accel_correction_hbf; 	//bias in accelerometer readings

	float k1_xy, k2_xy, k3_xy; 		/**< gain parameters for the complimentary filter of the inertial navigation */
	float k1_z, k2_z, k3_z; 		/**< gain parameters for the complimentary filter of the inertial navigation on z */

	uint32_t gps_last_update; 		/**< timestamp of the last update to the external postion tracking system #gps_last_update.*/
	uint32_t gps_last; 				/**< timestamp of the last update to the gps data #gps_last.*/
	uint8_t flag_gps_glitching;		/**< set to 1 if we have just recovered from a glitch in the GPS*/
	float lon_to_cm_scaling;         // conversion of longitude to centimeters
	int32_t last_good_lat;
	int32_t last_good_lng;
	uint32_t last_good_gps_update;

	uint32_t ext_pos_last_update;
	uint32_t ext_pos_last;
	uint8_t flag_ext_pos_glitching;
	Vector3f last_good_ext_pos;
	uint32_t last_good_ext_pos_update;

	float time_constant_xy;			/**< PARAM time constant for the gain parameters #time_constant_xy.*/
	float time_constant_z;			/**< PARAM time constant for the gain parameters #time_constant_z.*/

	// variables to store historic estimates calculated from the IMU
	uint8_t historic_xy_counter;
	float historic_x[AP_HISTORIC_XY_SIZE];
	float historic_y[AP_HISTORIC_XY_SIZE];
	Queue_property historic_x_property;
	Queue_property historic_y_property;

	// variables to store historic estimates calculated from the IMU
	uint8_t historic_z_counter;
	float historic_z[AP_HISTORIC_Z_SIZE];
	Queue_property historic_z_property;

}Inertial_nav_data;

/**
 * @brief update the INS system on the basis of new data
 * @param del_t time difference between two calls to the update function in ms
 */
void updateINAV(uint32_t del_t);

/**
 * @brief initialize the variables for inertial navigation
 */
void initINAV(void);

/**
 * @brief update the variables of the AHRS on the basis of new imu readings
 */
void updateAHRS(void);

/**
 * @brief automatically initialize the home position depending on the current position
 */
void initializeHome(void);

/**
 * @brief sets the position variables to home
 */
void setupHomePosition(void);

/**
 * @brief update the gains of the complementary filter used to update the inertial navigation data
 */
void updateGains(void);

/**
 * @brief sets the desired postion in the inertial navigation system units in cm
 */
void setPositionXY(float x, float y);

#endif /* INERTIAL_NAV_H_ */
