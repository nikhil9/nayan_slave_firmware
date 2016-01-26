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
#include "math.h"

#define CM_TO_MM 10
#define EPSILON 1e-3F

#define M_PI_F 3.141592653589793f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f


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
	uint32_t stamp; /* System Time of the instant at which data was obtained in  milliseconds */
	Vector3f accel; /* calibrated acceleration from IMU in meter per seconds in Body Frame*/
	Vector3f gyro; /* calibrated angular velocities from IMU in radian per seconds in Body Frame*/
	Vector3f attitude; /* attitude as measured in the body frame by imu in radians in Body Frame*/
}Sensor_IMU;


/**
 * @brief stores the gps latitude and longitude data along with the fused altitude from Barometer
 */
typedef struct
{
	uint32_t stamp;/* System Time of the instant at which data was obtained in  milliseconds */
	int32_t lat;/* Latitude * 1E07 from GPS */
	int32_t lng;/* Longitude * 1E07 from GPS */
	float alt;/* Relative Altitude from Barometer in cm*/
	Vector3f vel; /* Velocities from GPS in cm per sec in NED frame*/
}Sensor_Pose;

/**
 * @brief struct to acquire data from an external position sensor such as a vision based system
 * @note This is being updated at handleMessage() in OBC.c
 */
typedef struct
{
	uint32_t stamp; /* Slave Processor System timestamp */
	uint64_t obc_stamp; /* Timestamp from OBC */
	Vector3f attitude; /* Attitude (rpy) */
	Vector3f position; /* position in local frame */
	float yaw; /* yaw angle in NED frame */
	uint8_t flag_active; /* This is true if slave processor is receiving updates from OBC*/
	float depth; /* depth from Sonar if sent from OBC */
}Sensor_ExtPos;

/**
 * @brief struct to acquire setopints from On Board Computer
 * @note This is being updated at handleMessage() in OBC.c
 */
typedef struct
{
	uint32_t stamp; /* Slave Processor System timestamp */
	uint64_t obc_stamp; /* Timestamp from OBC */
	Vector3f position; /* position setpoint in user defined frame */
	Vector3f velocity; /* velocity setpoint in user defined frame */
	float yaw; /* yaw setpoint in rad*/
	float yaw_rate;/* yaw rate setpoint in rad/sec*/
}Setpoint_from_OBC;



extern Sensor_IMU sens_imu;

extern Sensor_Pose sens_pos;

extern Sensor_ExtPos sens_cv;

extern Setpoint_from_OBC set_point;

extern uint16_t rc_in[7];

extern bool_t dmc;

extern uint16_t control_command[4];

#endif /* MAIN_H_ */
