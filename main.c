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
float ang_vel[3];
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

int count_arming = 1000;
uint8_t FLAG_ARMING = 0;

int main(void){


	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	initINAV();

	//intialize the position_controller
	initializePosController();

	resetController();

	//intialize the wp_nav
	initializeWPNav();

	while(TRUE){
/**
 * USER CODE GOES HERE
 */
		uint32_t start = chTimeNow();
		if(sens_imu.stamp > last_imu_stamp)
		{
			if(isIMUGlitching() == 0)
			{
				updateAHRS();
				updateINAV(sens_imu.stamp - last_imu_stamp);
				last_imu_stamp = sens_imu.stamp;
			}
			else
			{
				delay(10);
				continue;
			}
		}

		//CONDITION FOR RUNNING LOITER to be run only when the switch is pressed on for the HLP code transfer
		//(Note that these values may need to recalibrated in case remote is changed)
		float chnl6_out = applyLPF(&wp_nav.channel6_filter, rc_in[6], 0.01);
		if(chnl6_out > (2000 + 917)/2)
			loiter_run();
		else
			resetController();

		//CONDITION FOR MOTORS BEING ARMED(Note that these values may need to recalibrated in case remote is changed)
		if(rc_in[2] < (THROTTLE_MIN + 80) && rc_in[3] > (STICK_MAX - 80))
		{
			if(count_arming < 100)			//pressed continuously for 1 sec @100Hz
				count_arming++;
		}
		else
		{
			if(count_arming > 0)
				count_arming--;
		}
		if(count_arming == 100 && FLAG_ARMING == 0)
		{
			FLAG_ARMING = 1;
			initINAV();						//need to reset the baro when arming
		}
		else if( count_arming == 0)
			FLAG_ARMING = 0;

		delay(10);
	}
	return 0;
}
