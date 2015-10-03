#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"
#include "inertial_nav.h"


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

vector_3f vis_pos_inp;
uint64_t prev_vis_inp_time;
uint64_t vis_inp_time;

vector_3f vis_pos;

vector_3f vis_vel;

Sensor_IMU sens_imu;
uint32_t last_imu_stamp;
Sensor_GPS sens_gps;
uint32_t last_gps_stamp;

AHRS ahrs;
Inertial_nav_data inav;

int main(void){


	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	while(TRUE){
/**
 * USER CODE GOES HERE
 */
		if(sens_imu.stamp > last_imu_stamp)
		{
			updateAHRS();
			updateINAV(sens_imu.stamp - last_imu_stamp);
		}
		if(vis_inp_time != prev_vis_inp_time)
		{
			vis_vel.x = vis_inp_time - prev_vis_inp_time;
			vis_vel.y = vis_pos_inp.y - vis_pos.y;
			vis_vel.z = vis_pos_inp.z - vis_pos.z;
			vis_pos = vis_pos_inp;
			vis_pos.y = millis();
			vis_pos.z = vis_vel.x;
			prev_vis_inp_time = vis_inp_time;
		}

		debug("MSG : RPY %f, %f ,%f", ahrs.attitude.x, ahrs.attitude.y, ahrs.attitude.z);

		vis_pos.x = (-123);
		vis_pos.y = (float)100;
		vis_pos.z = sens_imu.accel_calib.z;
		q[0] = 0.2;
		q[1] = 0.3;
		q[2] = 0.4;
		q[3] = 1;

		delay(10);
	}
	return 0;
}
