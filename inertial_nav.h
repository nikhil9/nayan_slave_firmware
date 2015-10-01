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

#ifndef INERTIAL_NAV_H_
#define INERTIAL_NAV_H_

#define GRAVITY_MSS 					9.80665f
#define GRAVITY_CMSS 					100*GRAVITY_MSS
#define M_PI_F 3.141592653589793f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define LATLON_TO_CM 1.113195f

#define INERTIAL_NAV_DELTAT_MAX 		0.1
#define AP_INTERTIALNAV_TC_XY   		2.5f // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    		5.0f // default time constant for complementary filter's Z axis
#define AP_INTERTIALNAV_GPS_TIMEOUT_MS  300  // timeout after which position error from GPS will fall to zero



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

	float time_constant_xy;			/**< PARAM time constant for the gain parameters #time_constant_xy.*/
	float time_constant_z;			/**< PARAM time constant for the gain parameters #time_constant_z.*/
//	uint32_t extpos1_last_update;
//	uint32_t extpos1_last;


}Inertial_nav_data;

extern Inertial_nav_data inav; /**< data structure storing the inertial navigation crucial data #inav. */

/**
 * @brief update the INS system on the basis of new data
 * @param del_t time difference between two calls to the update function in ms
 */
void updateINAV(uint32_t del_t);

/**
 * @brief initialize the variables for inertial navigation
 */
void init(void);

/**
 * @brief update the variables of the AHRS on the basis of new imu readings
 */
void updateAHRS(void);

/**
 * @brief set the Home position for the system
 * TODO not sure if required here(may only be required in the checkGPS position but still check it out
 */
void setHomePosition(uint32_t lat_home, uint32_t lng_home, uint32_t alt_home);

/**
 * @brief update the gains of the complementary filter used to update the inertial navigation data
 */
void updateGains(void);

/**
 * @brief sets the desired postion in the inertial navigation system units in cm
 * TODO
 */
void setPositionXY(float x, float y);

#endif /* INERTIAL_NAV_H_ */
