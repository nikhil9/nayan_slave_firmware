#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"

//variables for DEBUGGING via the mavlink message sim_state
float debug_vec[2];

/**
 * This variable contains velocity in centimeter per second
 * velocity.x -> velocity along x axis
 * velocity.y -> velocity along y axis
 * velocity.z -> velocity along z axis
 */
Vector3f velocity;

/**
 * This variable contains radio control input values.
 */
uint16_t rc_in[7];

float local_x_cm = 0;
float local_y_cm = 0;

Sensor_IMU sens_imu;
uint32_t last_imu_stamp;
Sensor_GPS sens_gps;
uint32_t last_gps_stamp;
Sensor_ExtPos sens_baro;
uint32_t last_baro_stamp;
Sensor_ExtPos sens_cv;
Sensor_Depth sens_sonar;

AHRS ahrs;
Inertial_nav_data inav;
Position_Controller pos_control;
WP_Nav wp_nav;

//User Defined Variables
Vector3f accel, gyro, attitude;
uint16_t rc_input_chn7;
Vector3f vision_pos;
Vector3f setpoint_obc;

int main(void)
{
	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	while(TRUE)
	{
		rc_input_chn7 = rc_in[6];

		if(rc_input_chn7 > 2100)
		{
			debug_vec[0] = 1.6;
			ic_rc_or_data.ic_rc.rc3 = 1600;				//overrides being sent to throttle
		}
		else
		{
			debug_vec[0] = 0;
			ic_rc_or_data.ic_rc.rc3 = 1500;				//overrides being sent to throttle
		}
		debug_vec[1] = -2.45;

		accel.x = sens_imu.accel_calib.x;				//acceleration outputs in m/s2 in NED body frame
		accel.y = sens_imu.accel_calib.y;				//acceleration outputs in m/s2 in NED body frame
		accel.z = sens_imu.accel_calib.z;				//acceleration outputs in m/s2 in NED body frame

		gyro.x = sens_imu.gyro_calib.x;					//angular velocity outputs in NED body frame in rad/s
		gyro.y = sens_imu.gyro_calib.y;					//angular velocity outputs in NED body frame in rad/s
		gyro.z = sens_imu.gyro_calib.z;					//angular velocity outputs in NED body frame in rad/s

		attitude.x = sens_imu.attitude.x;				//attitude Roll, Pitch and Yaw as calculated about NED frame in rad
		attitude.y = sens_imu.attitude.y;				//attitude Roll, Pitch and Yaw as calculated about NED frame in rad
		attitude.z = sens_imu.attitude.z;				//attitude Roll, Pitch and Yaw as calculated about NED frame in rad

		vision_pos.x = sens_cv.position.x;				// This is in NEU and in cm/s
		vision_pos.y = sens_cv.position.y;				// This is in NEU and in cm/s
		vision_pos.z = sens_cv.position.z;				// This is in NEU and in cm/s

		setpoint_obc.x = wp_nav.local_target_hbf.x;		// This is in NEU and in cm/s
		setpoint_obc.y = wp_nav.local_target_hbf.y;		// This is in NEU and in cm/s
		setpoint_obc.z = wp_nav.local_target_hbf.z;		// This is in NEU and in cm/s

		if(rc_in[0] > 1000 && rc_in[0] < 2000)
			ic_rc_or_data.ic_rc.rc1 = rc_in[0];			//overrides being sent to roll
		if(rc_in[1] > 1000 && rc_in[1] < 2000)
			ic_rc_or_data.ic_rc.rc2 = rc_in[1];			//overrides being sent to pitch
		if(rc_in[3] > 1000 && rc_in[3] < 2000)
			ic_rc_or_data.ic_rc.rc4 = rc_in[3];			//overrides being sent to yaw_rate

		delay(10);
	}
	return 0;
}
