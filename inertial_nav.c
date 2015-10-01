/*
 * inertial_nav.c
 *
 *  Created on: 29-Sep-2015
 *      Author: atulya
 */

#include <math.h>

#include "main.h"
#include "inertial_nav.h"

/**
 * @brief updates the secondary variables in AHRS assuming new imu input is available
 */
void updateAHRS(void)
{
	ahrs.cos_phi = cosf(ahrs.attitude.x);
	ahrs.sin_phi = sinf(ahrs.attitude.x);
	ahrs.cos_theta = cosf(ahrs.attitude.y);
	ahrs.sin_theta = sinf(ahrs.attitude.y);
	ahrs.cos_psi = cosf(ahrs.attitude.z);
	ahrs.sin_psi = sinf(ahrs.attitude.z);

	//TODO check out this formula from ArduCopter
	ahrs.accel_ef.x = ahrs.cos_theta*ahrs.cos_psi*sens_imu.accel_calib.x +
					(-ahrs.cos_phi*ahrs.sin_psi + ahrs.sin_phi*ahrs.sin_theta*ahrs.cos_psi)*sens_imu.accel_calib.y +
					(-ahrs.sin_phi*ahrs.sin_psi + ahrs.cos_phi*ahrs.sin_theta*ahrs.cos_psi)*sens_imu.accel_calib.z;

	ahrs.accel_ef.y = ahrs.cos_theta*ahrs.sin_psi*sens_imu.accel_calib.x +
					(ahrs.cos_phi*ahrs.cos_psi + ahrs.sin_phi*ahrs.sin_theta*ahrs.sin_psi)*sens_imu.accel_calib.y +
					(-ahrs.sin_phi*ahrs.cos_psi + ahrs.cos_phi*ahrs.sin_theta*ahrs.sin_psi)*sens_imu.accel_calib.z;

	ahrs.accel_ef.y = -ahrs.sin_theta*sens_imu.accel_calib.x +
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

/**
 * @brief sets the desired postion in the inertial navigation system units in cm
 */
void setPositionXY(float x, float y)
{
	inav.position_base.x = x;
	inav.position_base.y = y;
	inav.position_correction.x = 0.0f;
	inav.position_correction.y = 0.0f;

	//TODO clear historic estimates
	//add only last position to the historic estimate
}

/**
 * @brief corrects the IMU data so that all accelero and gyro are in the same frame
 * TODO check this out if it responds well
 */
static void correctIMUTemp(void)
{
	float tmp;
	tmp = sens_imu.accel_calib.y;
	sens_imu.accel_calib.y = -0.1*sens_imu.accel_calib.x;
	sens_imu.accel_calib.x = 0.1*tmp;
	sens_imu.accel_calib.z = -0.1*sens_imu.accel_calib.z;

	tmp = sens_imu.gyro_calib.y;
	sens_imu.gyro_calib.y = -0.1*sens_imu.gyro_calib.x;
	sens_imu.gyro_calib.x = 0.1*tmp;
	sens_imu.gyro_calib.z = -0.1*sens_imu.gyro_calib.z;

	ahrs.attitude.y = -ahrs.attitude.y;
	ahrs.attitude.z = -ahrs.attitude.z;
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

	//TODO check only the glitch condition where there is much error in position_base and position feedback
	if(isGPSGlitching())
	{
		setPositionXY(x,y);
		inav.position_error.x = 0;
		inav.position_error.y = 0;
	}
	else
	{
		Vector3f historic_position_base;

		//TODO correct on the basis of historic position update

		inav.position_error.x = x - (historic_position_base.x + inav.position_correction.x);
		inav.position_error.y = y - (historic_position_base.y + inav.position_correction.y);

	}

}

/**
 * @brief update navigation velocity and position based on GPS feedback if new available
 */
static void checkGPS(void)
{
	uint32_t now = millis();

	if(inav.gps_last > sens_gps.stamp)
	{
		float dt = (inav.gps_last - sens_gps.stamp)*0.001f;
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
			// increment error count TODO check if increment is really required from glitching function
//			if (_flags.ignore_error == 0 && _error_count < 255 && _xy_enabled) {
//				_error_count++;
		}
	}
}

/**
 * @brief initialize the variables of the navigation system
 */
void init()
{
	inav.time_constant_xy = AP_INTERTIALNAV_TC_XY;
	inav.time_constant_z = AP_INTERTIALNAV_TC_Z;
	updateGains();
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

	correctIMUTemp();		//TODO correct the readings of the IMU so they are all in the correct frame: remove this feature when slave code is rectified
	accel_ef = ahrs.accel_ef;	//TODO work this out properly ahrs acceleration is in dm/s/s so converting it to cm/s/s
	accel_ef.z += (-GRAVITY_CMSS);

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

	//TODO implement the historic base queue to take into effect the sensor delay
}
