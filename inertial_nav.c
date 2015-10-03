/*
 * inertial_nav.c
 *
 *  Created on: 29-Sep-2015
 *      Author: atulya
 */

#include <math.h>

#include "main.h"
#include "inertial_nav.h"

void updateAHRS(void)
{
	ahrs.cos_phi = cosf(ahrs.attitude.x);
	ahrs.sin_phi = sinf(ahrs.attitude.x);
	ahrs.cos_theta = cosf(ahrs.attitude.y);
	ahrs.sin_theta = sinf(ahrs.attitude.y);
	ahrs.cos_psi = cosf(ahrs.attitude.z);
	ahrs.sin_psi = sinf(ahrs.attitude.z);

	//DONEtodo- check out this formula from ArduCopter : present in AP_AHRS_DCM and AP_MATH/matrix3
	ahrs.accel_ef.x = ahrs.cos_theta*ahrs.cos_psi*sens_imu.accel_calib.x +
					(-ahrs.cos_phi*ahrs.sin_psi + ahrs.sin_phi*ahrs.sin_theta*ahrs.cos_psi)*sens_imu.accel_calib.y +
					(ahrs.sin_phi*ahrs.sin_psi + ahrs.cos_phi*ahrs.sin_theta*ahrs.cos_psi)*sens_imu.accel_calib.z;

	ahrs.accel_ef.y = ahrs.cos_theta*ahrs.sin_psi*sens_imu.accel_calib.x +
					(ahrs.cos_phi*ahrs.cos_psi + ahrs.sin_phi*ahrs.sin_theta*ahrs.sin_psi)*sens_imu.accel_calib.y +
					(-ahrs.sin_phi*ahrs.cos_psi + ahrs.cos_phi*ahrs.sin_theta*ahrs.sin_psi)*sens_imu.accel_calib.z;

	ahrs.accel_ef.z = -ahrs.sin_theta*sens_imu.accel_calib.x +
					ahrs.sin_phi*ahrs.cos_theta*sens_imu.accel_calib.y +
					ahrs.cos_phi*ahrs.cos_theta*sens_imu.accel_calib.z;

}

/**
 * @brief returns whether the GPS is having any glitch
 * TODO
 */
static int isGPSGlitching(void)
{
	return 0;
}

void setPositionXY(float x, float y)
{
	inav.position_base.x = x;
	inav.position_base.y = y;
	inav.position_correction.x = 0.0f;
	inav.position_correction.y = 0.0f;

	 // clear historic estimates
	resetQueue(inav.historic_x, &inav.historic_x_property);
	resetQueue(inav.historic_y, &inav.historic_y_property);

	// add new position for future use
	inav.historic_xy_counter = 0;
	pushToQueue(inav.position_base.x, inav.historic_x, &inav.historic_x_property);
	pushToQueue(inav.position_base.y, inav.historic_y, &inav.historic_y_property);
}

/**
 * @brief correct the inav with new gps updates
 */
static void correctWithGPS(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

	float x = (sens_gps.lat - ahrs.lat_home) * LATLON_TO_CM;
	float y = (sens_gps.lng - ahrs.lng_home) * cosf(sens_gps.lat * 1.0e-7f * DEG_TO_RAD) * LATLON_TO_CM;

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	if(isGPSGlitching())
	{
		// failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
		inav.position_error.x *= 0.7943f;
		inav.position_error.y *= 0.7943f;
	}
	else
	{
		// if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
		// reset the inertial nav position and velocity to gps values
		if(inav.flag_gps_glitching == 1)
		{
			setPositionXY(x,y);
			inav.position_error.x = 0.0f;
			inav.position_error.y = 0.0f;
		}
		else
		{
			Vector3f historic_position_base;

			if(inav.historic_x_property.is_full)
			{
				historic_position_base.x = popQueue(inav.historic_x, &inav.historic_x_property);
				historic_position_base.y = popQueue(inav.historic_y, &inav.historic_y_property);
			}
			else
			{
				historic_position_base.x = inav.position_base.x;
				historic_position_base.y = inav.position_base.y;
			}

			inav.position_error.x = x - (historic_position_base.x + inav.position_correction.x);
			inav.position_error.y = y - (historic_position_base.y + inav.position_correction.y);
		}
	}
	inav.flag_gps_glitching = isGPSGlitching();

}

/**
 * @brief update navigation velocity and position based on GPS feedback if new available
 */
static void checkGPS(void)
{
	uint32_t now = millis();

	if( sens_gps.stamp > inav.gps_last)
	{
		float dt = (sens_gps.stamp - inav.gps_last)*0.001f;
		inav.gps_last_update = now;
		correctWithGPS(dt);
		inav.gps_last = sens_gps.stamp;
	}
	else
	{
		// if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
		if (now - inav.gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		{
			inav.position_error.x *= 0.9886f;
			inav.position_error.y *= 0.9886f;
		}
	}
}

void initializeHome()
{
	//wait for some proper measurement from the GPS
	while(sens_gps.stamp == 0)
		delay(10);

	//as soon as a proper GPS lock signal is obtained
	ahrs.lat_home = sens_gps.lat;
	ahrs.lng_home = sens_gps.lng;

	setupHomePosition();
}

void setupHomePosition()
{
	// reset corrections to base position to zero
	inav.position_base.x = 0.0f;
	inav.position_base.y = 0.0f;
	inav.position_correction.x = 0.0f;
	inav.position_correction.y = 0.0f;
	inav.position.x = 0.0f;
	inav.position.y = 0.0f;

	inav.position_error.x = 0.0f;
	inav.position_error.y = 0.0f;
	inav.velocity.x = 0;
	inav.velocity.y = 0;

	// clear historic estimates
	resetQueue(inav.historic_x, &inav.historic_x_property);
	resetQueue(inav.historic_y, &inav.historic_y_property);

}

void init()
{
	// initialize the queues
	inav.historic_xy_counter = 0;
	inav.historic_z_counter = 0;
	inav.historic_x_property.size = AP_HISTORIC_XY_SIZE;
	inav.historic_y_property.size = AP_HISTORIC_XY_SIZE;
	inav.historic_z_property.size = AP_HISTORIC_Z_SIZE;
	resetQueue(inav.historic_x, &inav.historic_x_property);
	resetQueue(inav.historic_y, &inav.historic_y_property);
	resetQueue(inav.historic_z, &inav.historic_z_property);

	//update the gain parameters
	inav.time_constant_xy = AP_INTERTIALNAV_TC_XY;
	inav.time_constant_z = AP_INTERTIALNAV_TC_Z;
	updateGains();

	//update gps time variables
	inav.gps_last = 0;
	inav.gps_last_update = 0;

	initializeHome();
	//TODO initialize other variables
	//acceleration_correction_hbf remaining :  can be initialized by considering some intial values OR maybe be left to converge
	//z components of position, position_correction, position_base, position_error and velocity remaining
	//gps_last and gps_last_update are remaining

}

void updateGains()
{
	// X & Y axis time constant
	if (inav.time_constant_xy == 0.0f)
	{
		inav.k1_xy = inav.k2_xy = inav.k3_xy = 0.0f;
	}
	else
	{
		inav.k1_xy = 3.0f / inav.time_constant_xy;
		inav.k2_xy = 3.0f / (inav.time_constant_xy*inav.time_constant_xy);
		inav.k3_xy = 1.0f / (inav.time_constant_xy*inav.time_constant_xy*inav.time_constant_xy);
	}

	// Z axis time constant
	if (inav.time_constant_z == 0.0f)
	{
		inav.k1_z = inav.k2_z = inav.k3_z = 0.0f;
	}
	else
	{
		inav.k1_z = 3.0f / inav.time_constant_z;
		inav.k2_z = 3.0f / (inav.time_constant_z*inav.time_constant_z);
		inav.k3_z = 1.0f / (inav.time_constant_z*inav.time_constant_z*inav.time_constant_z);
	}
}

void updateINAV(uint32_t del_t)
{
	float dt = del_t * 0.001f;

	if(dt > INERTIAL_NAV_DELTAT_MAX)
		return;

	// check if new gps readings have arrived and use them to correct position estimates
	checkGPS();

	Vector3f accel_ef;

	accel_ef.x = ahrs.accel_ef.x*100;	//acceleration in ahrs obtained is in mss and we are storing in cmss
	accel_ef.y = ahrs.accel_ef.y*100;	//acceleration in ahrs obtained is in mss and we are storing in cmss

	accel_ef.z = -(ahrs.accel_ef.z*100 + GRAVITY_CMSS); //converting acceleration from NED to NEU for proper calculation of altitude

	//position is assumed with respect to an NEU frame
	// convert ef position error to horizontal body frame
	Vector2f position_error_hbf;
	position_error_hbf.x = inav.position_error.x * ahrs.cos_psi + inav.position_error.y * ahrs.sin_psi;
	position_error_hbf.y = -inav.position_error.x * ahrs.sin_psi + inav.position_error.y * ahrs.cos_psi;

	float tmp = inav.k3_xy * dt;
	inav.accel_correction_hbf.x += position_error_hbf.x * tmp;
	inav.accel_correction_hbf.y += position_error_hbf.y * tmp;
	inav.accel_correction_hbf.z += inav.position_error.z * inav.k3_z  * dt;

	tmp = inav.k2_xy * dt;
	inav.velocity.x += inav.position_error.x * tmp;
	inav.velocity.y += inav.position_error.y * tmp;
	inav.velocity.z += inav.position_error.z * inav.k2_z  * dt;

	tmp = inav.k1_xy * dt;
	inav.position_correction.x += inav.position_error.x * tmp;
	inav.position_correction.y += inav.position_error.y * tmp;
	inav.position_correction.z += inav.position_error.z * inav.k1_z  * dt;

	// convert horizontal body frame accel correction to earth frame
	Vector2f accel_correction_ef;
	accel_correction_ef.x = inav.accel_correction_hbf.x * ahrs.cos_psi - inav.accel_correction_hbf.y * ahrs.sin_psi;
	accel_correction_ef.y = inav.accel_correction_hbf.x * ahrs.sin_psi + inav.accel_correction_hbf.y * ahrs.cos_psi;

	// calculate velocity increase adding new acceleration from accelerometers
	Vector3f velocity_increase;
	velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
	velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
	velocity_increase.z = (accel_ef.z + inav.accel_correction_hbf.z) * dt;

	// calculate new estimate of position
	inav.position_base.x += (inav.velocity.x + velocity_increase.x*0.5) * dt;
	inav.position_base.y += (inav.velocity.y + velocity_increase.y*0.5) * dt;
	inav.position_base.z += (inav.velocity.z + velocity_increase.z*0.5) * dt;

	// update the corrected position estimate
	inav.position.x = inav.position_base.x + inav.position_correction.x;
	inav.position.y = inav.position_base.y + inav.position_correction.y;
	inav.position.z = inav.position_base.z + inav.position_correction.z;

	// calculate new velocity
	inav.velocity.x += velocity_increase.x;
	inav.velocity.y += velocity_increase.y;
	inav.velocity.z += velocity_increase.z;

	//TODO CHECK the working
	// store 3rd order estimate (i.e. estimated vertical position) for future use
	pushToQueue(inav.position_base.z, inav.historic_z, &inav.historic_z_property);

	// store 3rd order estimate (i.e. horizontal position) for future use at 10hz
	inav.historic_xy_counter++;
	if( inav.historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS )
	{
		inav.historic_xy_counter = 0;
		pushToQueue(inav.position_base.x, inav.historic_x, &inav.historic_x_property);
		pushToQueue(inav.position_base.y, inav.historic_y, &inav.historic_y_property);
	}

}
