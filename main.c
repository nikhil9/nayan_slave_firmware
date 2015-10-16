#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"

//variables for state debugging via the mavlink message sim_state
float q[4];
/**
 * This variable contains velocity in centimeter per second
 * velocity.x -> velocity along x axis
 * velocity.y -> velocity along y axis
 * velocity.z -> velocity along z axis
 */
vector_3f velocity;


/**
 * This variable contains radio control input values.
 */
uint16_t rc_in[7];

/**
 * @warning DO NOT EDIT!
 * main() function with system initialization
 * @return 0
 */


float x_cm = 0;
float y_cm = 0;

vector_3f vis_pos_inp;
uint64_t prev_vis_inp_time;
uint64_t vis_inp_time;

vector_3f vis_pos;

vector_3f vis_vel;

Sensor_IMU sens_imu;
uint32_t last_imu_stamp;
Sensor_GPS sens_gps;
uint32_t last_gps_stamp;
Sensor_ExtPos sens_ext_pos;
uint32_t last_ext_pos_stamp;

AHRS ahrs;
Inertial_nav_data inav;
Position_Controller pos_control;
WP_Nav wp_nav;

int main(void){


	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	initINAV();

	//intialize the position_controller
	initializePosController();

	//intialize the wp_nav
	initializeWPNav();

	while(TRUE){
/**
 * USER CODE GOES HERE
 */
		uint32_t start = micros();
		if(sens_imu.stamp > last_imu_stamp)
		{
			updateAHRS();
			updateINAV(sens_imu.stamp - last_imu_stamp);
			last_imu_stamp = sens_imu.stamp;
		}

		loiter_run();

//		debug("absolute values is : %d", abs(-15));

		if(sens_ext_pos.stamp > last_ext_pos_stamp)
		{
			debug("Yaw from vision is : %f and data is updated at %dms", sens_ext_pos.yaw, sens_ext_pos.stamp - last_ext_pos_stamp);
			last_ext_pos_stamp = sens_ext_pos.stamp;
		}

		//debug("MSG : RPY %f, %f ,%f", ahrs.attitude.x, ahrs.attitude.y, ahrs.attitude.z);
		uint32_t end = micros();

//		debug("lat home is: %d; lng_home is: %d", ahrs.lat_home, ahrs.lng_home);
		int32_t duration = (end - start);				// time for which this thread should sleep
//		debug("Duration of the code execution is %d us", duration);

		delay(2);
	}
	return 0;
}
