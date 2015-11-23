#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"

//variables for state DEBUGGING via the mavlink message sim_state
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

int count_arming = 1000;
uint8_t FLAG_ARMING = 0;

inline void checkArmingStatus(void)
{
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
}

int main(void)
{
	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();
	initializePosController();
	resetController();
	initializeWPNav();
	initINAV();

	while(TRUE)
	{
		uint32_t start = chTimeNow();
		if(sens_imu.stamp > last_imu_stamp)
		{
			debug("IMU : %d,%f,%f,%f",sens_imu.stamp, sens_imu.attitude.x, sens_imu.attitude.y, sens_imu.attitude.z);
			debug("IMU : %d,%f,%f,%f",ahrs.stamp, ahrs.attitude.x, ahrs.attitude.y, ahrs.attitude.z);
			if(isIMUGlitching() == 0)
			{
				updateAHRS();
				updateINAV(sens_imu.stamp - last_imu_stamp);
				last_imu_stamp = sens_imu.stamp;
			}
			else
			{
				debug("IMU glitched");
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

		checkArmingStatus();

		if(count_arming == 100 && FLAG_ARMING == 0)
		{
			FLAG_ARMING = 1;
			resetINAV();						//need to reset the baro when arming
		}
		else if( count_arming == 0)
			FLAG_ARMING = 0;

		delay(10);

		uint32_t stop = chTimeNow();
		float duration = (stop-start)/0.1f;
//		debug("Time for execution of code is %f us", duration);
	}
	return 0;
}
