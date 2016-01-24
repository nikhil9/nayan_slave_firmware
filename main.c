#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"


//Handles imu data
Sensor_IMU sens_imu;

//Handles position data from GPS and Baro
Sensor_Pose sens_pos;

//Handles position data from Odroid. This is being updated at handleMessage()
//function in odroid.c
Sensor_ExtPos sens_cv;

//Handles radio control inputs
uint16_t rc_in[7];

//If TRUE -> control_command control motors directly (This will bypass Master Controller) (Use Very Cautiously)
//if FALSE-> control_command are setpoints for attitude controller running on Master Processor
bool_t dmc = FALSE;

/**
 * This array is passed to Master controller for Motor Control.
 * Update these variables with desired value as Output of your control Algorithm
 *
 * If dmc = TRUE,
 * control_command[0] -> Motor 1
 * control_command[1] -> Motor 2
 * control_command[2] -> Motor 3
 * control_command[3] -> Motor 4
 *
 * If dmc = FALSE,
 * control_command[0] -> Desired Roll		(Range: 1000 to 2000)(Scaling -30deg to +30deg)
 * control_command[1] -> Desired Pitch		(Range: 1000 to 2000)(Scaling -30deg to +30deg)
 * control_command[2] -> Desired Throttle
 * control_command[3] -> Desired Yaw		(Range: 1000 to 2000)(Scaling -90deg/sec to +90deg/sec)
 *
 */
uint16_t control_command[4] = {1500, 1500, 1000, 1500};



/**
 * <-- Master Control Setpoints -->
 * This function maps radio control values from master processor to control commands.
 * When switch to Slave Processor Nayan will fly similar to Manual Mode.
*/
void example_function_1(void){

	dmc = FALSE;
	control_command[0] = rc_in[0];
	control_command[1] = rc_in[1];
	control_command[2] = rc_in[2];
	control_command[3] = rc_in[3];
}

/**
 * <-- Direct Motor Control -->
 * Throttle value from RC controller will be given directly to all 4 motors
 * This function is for testing purpose only, One should not use this for flying
 *
*/
void example_function_2(void){

	dmc = TRUE;
	control_command[0] = rc_in[2];
	control_command[1] = rc_in[2];
	control_command[2] = rc_in[2];
	control_command[3] = rc_in[2];
}


int main(void)
{
	halInit();
	chSysInit();

	start_sys();

	delay(1000);

	odroid_comm_init();

	while(TRUE)
	{

		/**
		 * User code goes here. This loop will be executed at 100Hz
		 */

//		example_function_1();

//		example_function_2();


		delay(100);
	}
	return 0;
}
