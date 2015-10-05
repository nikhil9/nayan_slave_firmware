/*
 * main.h
 *
 *  Created on: Aug 2, 2014
 *      Author: nikhil
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"
#include "hal.h"
#include "Setup.h"
#include "intercomm.h"
#include "autopilot_math.h"

/**
 * @brief stores the raw imu variables acceleration and the angular velocity
 */
typedef struct
{
	uint32_t stamp; /**> timestamp of the instant data was obtained #stamp.*/
	Vector3f accel_calib; /**> acceleration as measured by the accelerometer and calibrated #accel_calib.*/
	Vector3f gyro_calib; /**> angular velocity as measured in the body frame by imu #angular_velocity.*/
}Sensor_IMU;

/**
 * @brief stores the gps latitude and longitude data along with the fused altitude
 */
typedef struct
{
	uint32_t stamp;
	int32_t lat;
	int32_t lng;
	float alt;
}Sensor_GPS;

/**
 * @brief struct to acquire data from an external position sensor such as a vision based system
 */
typedef struct
{
	uint32_t stamp;
	uint64_t obc_stamp;
	Vector3f position;
	float yaw;
}Sensor_ExtPos;
/**
 * @brief stores the current attitude and the trigonometric values for future use
 */
typedef struct
{
	uint32_t stamp;
	Vector3f attitude;

	int32_t lat_home;
	int32_t lng_home;
	float alt_home;

	float sin_phi, cos_phi;
	float sin_theta, cos_theta;
	float sin_psi, cos_psi;

	Vector3f accel_ef;
}AHRS;

extern Sensor_IMU sens_imu; /**< struct holding current imu variables #sens_imu.*/

extern Sensor_GPS sens_gps;

extern Sensor_ExtPos sens_ext_pos;

extern AHRS ahrs;

extern vector_3f velocity;
extern uint16_t rc_in[7];

//variables for sim_state
extern float q[4];

//TODO remove the temp variables after use
extern float x_cm;
extern float y_cm;

#endif /* MAIN_H_ */
