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

typedef struct{
	float x;
	float y;
	float z;

}vector_3f;

/**
 * @brief implements a 2 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
}Vector2f;

/**
 * @brief implements a 3 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
	float z;
}Vector3f;

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
	float lat;
	float lng;
	float alt;
}Sensor_GPS;

/**
 * @brief stores the current attitude and the trigonometric values for future use
 */
typedef struct
{
	uint32_t stamp;
	Vector3f attitude;

	float lat_home;
	float lng_home;

	float sin_phi, cos_phi;
	float sin_theta, cos_theta;
	float sin_psi, cos_psi;

	Vector3f accel_ef;
}AHRS;

extern Sensor_IMU sens_imu; /**< struct holding current imu variables #sens_imu.*/

extern Sensor_GPS sens_gps;

extern AHRS ahrs;

extern vector_3f velocity;
extern uint16_t rc_in[7];

//variables for control_system_state
extern float vel_variance[3];
extern float pos_variance[3];
extern float q[4];

/* Extra variables added by atulya for communication with odroid*/
/*--------------------------------------------------------------*/
extern vector_3f vis_pos_inp;
extern uint64_t prev_vis_inp_time;
extern uint64_t vis_inp_time;

extern vector_3f vis_pos;

extern vector_3f vis_vel;
/*--------------------------------------------------------------*/

#endif /* MAIN_H_ */
