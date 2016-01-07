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
#include "inertial_nav.h"
#include "position_controller.h"
#include "wp_nav.h"

#define M_PI_F 3.141592653589793f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define ARMING_COUNT 175

/**
 * @brief stores the raw imu variables acceleration and the angular velocity
 */
typedef struct
{
	uint32_t stamp; /**> timestamp of the instant data was obtained #stamp.*/
	Vector3f accel_calib; /**> acceleration as measured by the accelerometer and calibrated #accel_calib.*/
	Vector3f gyro_calib; /**> angular velocity as measured in the body frame by imu #angular_velocity.*/
	Vector3f attitude; /**> attitude as measured in the body frame by imu #angular_velocity.*/
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
	uint8_t flag_active;
}Sensor_ExtPos;

typedef struct
{
	uint32_t stamp;
	uint64_t obc_stamp;
	float depth;
}Sensor_Depth;
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
extern Sensor_ExtPos sens_baro;
extern Sensor_ExtPos sens_cv;
extern Sensor_Depth sens_sonar;

extern AHRS ahrs;
extern Inertial_nav_data inav; /**< data structure storing the inertial navigation crucial data #inav. */
extern Position_Controller pos_control;
extern WP_Nav wp_nav;

extern Vector3f velocity;
extern uint16_t rc_in[7];

//variables for debugging sent through sim_state
extern float debug_vec[2];

//TODO Phase B : remove the temp variables after improving architecture
extern float local_x_cm;
extern float local_y_cm;

extern Vector3f accel, gyro, attitude;
extern uint16_t rc_input_chn6;
extern Vector3f vision_pos;
extern Vector3f setpoint_obc;


#endif /* MAIN_H_ */
