#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "Setup.h"
#include "intercomm.h"
#include "odroid_comm.h"


/**
 * This variable contains acceleration values in g
 * accel.x -> acceleration along x axis
 * accel.y -> acceleration along y axis
 * accel.z -> acceleration along z axis
 */
vector_3f accel;

/**
 * This variable contains angular rate values in radians per second
 * gyro.x -> angular rate about x axis
 * gyro.y -> angular rate about y axis
 * gyro.z -> angular rate about z axis
 */
vector_3f gyro;


/**
 * This variable contains attitude values in radians
 * attitude.x -> Roll
 * attitude.y -> Pitch
 * attitude.z -> Heading
 */
vector_3f attitude;

/**
 * This variable contains location estimates
 * position.x -> Lattitude * 1E07
 * position.y -> Longitude * 1E07
 * position.z -> Altitude (Relative)
 */
vector_3f position;

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
			delay(20);

	}
	return 0;
}
