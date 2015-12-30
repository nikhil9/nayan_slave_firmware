/*
 * inertial_nav.c
 *
 *  Created on: 29-Sep-2015
 *      Author: atulya
 */


#include "main.h"

static void printQueue(float arr[], Queue_property q_property)
{
	debug("Size = %d ; Front = %d; Last = %d; is_full = %d; is_empty = %d\n",
		q_property.size, q_property.first, q_property.last, q_property.is_full, q_property.is_empty);

	int i;
	for(i=0; i<q_property.size; i++)
		debug("%1.2f ", arr[i]);
	debug("\n");
}

int isIMUGlitching(void)
{
	int all_ok = 1;

	if(isnan(sens_imu.attitude.x)||isnan(sens_imu.attitude.y)||isnan(sens_imu.attitude.z))
		all_ok = 0;

	if(isinf(sens_imu.attitude.x)||isinf(sens_imu.attitude.y)||isinf(sens_imu.attitude.z))
		all_ok = 0;

	//assuming that the maximum attitude angles is 10 rad
	if((fabs(sens_imu.attitude.x)>10)||(fabs(sens_imu.attitude.y)>10)||(fabs(sens_imu.attitude.z)>10))
		all_ok = 0;

	if(isnan(sens_imu.accel_calib.x)||isnan(sens_imu.accel_calib.y)||isnan(sens_imu.accel_calib.z))
		all_ok = 0;

	if(isinf(sens_imu.accel_calib.x)||isinf(sens_imu.accel_calib.y)||isinf(sens_imu.accel_calib.z))
		all_ok = 0;

	//assuming that a sane accelerometer reading + g  is at least 1mss
	if(normVec3f(sens_imu.accel_calib) < MIN_ACCEL_MEASURED)
		all_ok = 0;

	Vector3f accel_diff;
	accel_diff.x = sens_imu.accel_calib.x - inav.last_good_imu.x;
	accel_diff.y = sens_imu.accel_calib.y - inav.last_good_imu.y;
	accel_diff.z = sens_imu.accel_calib.z - inav.last_good_imu.z;

//	q[1] = normVec3f(accel_diff);

//	if(normVec3f(accel_diff) > MAX_ACCEL_CHANGE)
//		all_ok = 0;

	if(all_ok == 1 )
	{
		inav.last_good_imu = sens_imu.accel_calib;
		inav.last_good_imu_update = sens_imu.stamp;
	}
	else
	{
		debug("IMU glitched");
		debug("IMU LLP : %d,%f,%f,%f",sens_imu.stamp, sens_imu.attitude.x, sens_imu.attitude.y, sens_imu.attitude.z);
		debug("AHRS : %d,%f,%f,%f",ahrs.stamp, ahrs.attitude.x, ahrs.attitude.y, ahrs.attitude.z);
	}

	return (!all_ok);
}

void updateAHRS(void)
{
	ahrs.stamp = sens_imu.stamp;
	ahrs.attitude.x = sens_imu.attitude.x;
	ahrs.attitude.y = sens_imu.attitude.y;
#if (USE_GPS_NOT_CV == 1)
		ahrs.attitude.z = sens_imu.attitude.z;
#else
    	ahrs.attitude.z = sens_cv.yaw;
#endif


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

// set_altitude - set base altitude estimate in cm
void setAltitude( float new_altitude)
{
    inav.position_base.z = new_altitude;
    inav.position_correction.z = 0;
    inav.position.z = new_altitude; // _position = _position_base + _position_correction
    resetQueue(inav.historic_z, &inav.historic_z_property);

    inav.historic_z_counter = 0;
    pushToQueue(inav.position_base.z, inav.historic_z, &inav.historic_z_property);
}

/**
 * @brief returns whether the GPS is having any glitch
 */

static int isGPSGlitching(void)
{
// calculate time since last sane gps reading in ms
	float sane_dt = (sens_gps.stamp - inav.last_good_gps_update) / 1000.0f;

	float dlat = sens_gps.lat - inav.last_good_lat;
	float dlong = sens_gps.lng - inav.last_good_lng;

	float distance_cm = sqrt(dlat*dlat + dlong*dlong)*LATLON_TO_CM;
//	debug("distance to last_good_lat is %.2f", distance_cm);

	int all_ok = 1;

	if(isnan(sens_gps.lat) || isnan(sens_gps.lng))
		all_ok = 0;

	if(isinf(sens_gps.lat) || isinf(sens_gps.lng))
		all_ok = 0;

	    // all ok if within a given hardcoded radius
	if (distance_cm > GPS_RADIUS_CM)
	{
		all_ok = 0;
	}

	if(all_ok == 1)
	{
		inav.last_good_lat = sens_gps.lat;
		inav.last_good_lng = sens_gps.lng;
		inav.last_good_gps_update = sens_gps.stamp;
	}
	else
	{
		debug("GPS glitched. previous is (%f, %f); Current is (%f, %f)",
										inav.last_good_lat*1e-7, inav.last_good_lng*1e-7,
										sens_gps.lat*1e-7, sens_gps.lng*1e-7);
	}
	return (!all_ok);
}


/**
 * @brief correct the inav with new gps updates
 */
static void correctWithGPS(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

	local_x_cm = (sens_gps.lat - ahrs.lat_home) * LATLON_TO_CM;
	local_y_cm = (sens_gps.lng - ahrs.lng_home) * inav.lon_to_cm_scaling;

//	debug("GPS lat is %d; GPS lng is %d;", sens_gps.lat, sens_gps.lng);
//	debug("GPS x is %.2f; GPS y is %.2f; deltat is %.2f",x,y,dt);

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	int glitching_status = isGPSGlitching();
	if(glitching_status)
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
			debug("position base %.2f; position correction is %.2f; position_error is %.2f",
							inav.position_base.x, inav.position_correction.x, inav.position_error.x);
			setPositionXY(local_x_cm,local_y_cm);
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

//			debug("position from gps is %.2f; historic_position_base is %.2f; position_correction is %.2f",
//							local_x_cm, inav.position_base.x, inav.position_correction.x);

			inav.position_error.x = local_x_cm - (historic_position_base.x + inav.position_correction.x);
			inav.position_error.y = local_y_cm - (historic_position_base.y + inav.position_correction.y);
		}
	}
	inav.flag_gps_glitching = glitching_status;

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
/**
 * @brief checks if the external position sensor is glitching
 */
static int isCVGlitching(void)
{
// calculate time since last sane gps reading in ms
	float sane_dt = (sens_cv.stamp - inav.last_good_gps_update) / 1000.0f;

	float dx = sens_cv.position.x - inav.last_good_cv.x;
	float dy = sens_cv.position.y - inav.last_good_cv.y;

	float distance_cm = sqrt(dx*dx + dy*dy);
//	debug("distance to last_good_lat is %.2f", distance_cm);

	int all_ok = 1;

	if(isnan(sens_cv.position.x) || isnan(sens_cv.position.y))
		all_ok = 0;

	if(isinf(sens_cv.position.x) || isinf(sens_cv.position.y))
		all_ok = 0;

	    // all ok if within a given hardcoded radius
	if (distance_cm > GPS_RADIUS_CM)
	{
		all_ok = 0;
	}

	if(all_ok == 1)
	{
		inav.last_good_cv.x = sens_cv.position.x;
		inav.last_good_cv.y = sens_cv.position.y;
		inav.last_good_cv_update = sens_cv.stamp;
	}
	else
	{
		debug("CV glitched. previous CV  is (%.2f, %.2f); Current CV is (%.2f, %.2f)",
												inav.last_good_cv.x, inav.last_good_cv.y,
												sens_cv.position.x, sens_cv.position.y);
	}
	return (!all_ok);
}


/**
 * @brief correct the inav with new gps updates
 */
static void correctWithCV(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

	local_x_cm = sens_cv.position.x;
	local_y_cm = sens_cv.position.y;

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	int glitching_status = isCVGlitching();
	if(glitching_status)
	{
		// failed sanity check so degrate position_error to 10% over 2 seconds (assumes 30hz update rate)
		inav.position_error.x *= 0.9624f;
		inav.position_error.y *= 0.9624f;
	}
	else
	{
		// if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
		// reset the inertial nav position and velocity to gps values
		if(inav.flag_cv_glitching == 1)
		{
			debug("position base %.2f; position correction is %.2f; position_error is %.2f",
							inav.position_base.x, inav.position_correction.x, inav.position_error.x);
			setPositionXY(local_x_cm,local_y_cm);
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

//			debug("position from gps is %.2f; historic_position_base is %.2f; position_correction is %.2f",
//							local_x_cm, inav.position_base.x, inav.position_correction.x);

			inav.position_error.x = local_x_cm - (historic_position_base.x + inav.position_correction.x);
			inav.position_error.y = local_y_cm - (historic_position_base.y + inav.position_correction.y);
		}
	}
	inav.flag_cv_glitching = glitching_status;

}

/**
 * @brief update navigation velocity and position based on GPS feedback if new available
 */
static void checkCV(void)
{
	uint32_t now = millis();

	if( sens_cv.stamp > inav.cv_last)
	{
		float dt = (sens_cv.stamp - inav.cv_last)*0.001f;
		inav.cv_last_update = now;
		correctWithCV(dt);
		inav.cv_last = sens_cv.stamp;
	}
	else
	{
		// if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 30hz update rate)
		if (now - inav.cv_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		{
			inav.position_error.x *= 0.9624f;
			inav.position_error.y *= 0.9624f;
		}
	}
}
/**
 * @brief checks if the external position sensor is glitching
 */
static int isBaroGlitching(void)
{
	// calculate time since last sane gps reading in ms
	float sane_dt = (sens_baro.stamp - inav.last_good_gps_update) / 1000.0f;

	float distance_cm = sens_baro.position.z - inav.last_good_baro.z;
//	debug("distance to last_good_baro is %.2f", distance_cm);

	int all_ok = 1;
//	q[2] = distance_cm;
		// all ok if within a given hardcoded radius

	if(isnan(sens_baro.position.z))
		all_ok = 0;

	if(isinf(sens_baro.position.z))
		all_ok = 0;

	if (fabs(distance_cm) > BARO_RADIUS_CM)
	{
		all_ok = 0;
	}

	if(sens_baro.position.z == 0)
	{
		all_ok = 0;
	}

	if(all_ok == 1)
	{
		inav.last_good_baro.z = sens_baro.position.z;
		inav.last_good_baro_update = sens_baro.stamp;
	}
	else
	{
		debug("BARO glitched. previous baro reading is %.2f; Current baro reading is %.2f", inav.last_good_baro.z, sens_baro.position.z);
	}
	return (!all_ok);
}


/**
 * @brief correct the inav with updates from the external navigation sensor(vision/px4flow)
 */
static void correctWithBaro(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

//	local_x_cm = sens_baro.position.x*100;
//	local_y_cm = sens_baro.position.y*100;
	float z = sens_baro.position.z;

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	int glitching_status = isBaroGlitching();
	if(glitching_status)
	{
		// failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
//		inav.position_error.x *= 0.7943f;
//		inav.position_error.y *= 0.7943f;
		inav.position_error.z *= 0.8859f;
	}
	else
	{
		// if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
		// reset the inertial nav position and velocity to gps values
		if(inav.flag_baro_glitching == 1)
		{
			debug("Z : baro: %z pos base %.2f; pos_correction is %.2f; pos_error is %.2f",
							z, inav.position_base.z, inav.position_correction.z, inav.position_error.z);

//			setPositionXY(local_x_cm,local_y_cm);
			setAltitude(z);
			inav.position_error.z = 0;
		}
		else
		{
			Vector3f historic_position_base;

//			if(inav.historic_x_property.is_full)
//			{
//				historic_position_base.x = popQueue(inav.historic_x, &inav.historic_x_property);
//				historic_position_base.y = popQueue(inav.historic_y, &inav.historic_y_property);
//			}
//			else
//			{
//				historic_position_base.x = inav.position_base.x;
//				historic_position_base.y = inav.position_base.y;
//			}
			if(inav.historic_z_property.is_full)
				historic_position_base.z = popQueue(inav.historic_z, &inav.historic_z_property);
			else
				historic_position_base.z = inav.position_base.z;

//			debug("position from baro is %.2f; historic_position_base is %.2f; position_correction is %.2f",
//							z, inav.position_base.z, inav.position_correction.z);

//			inav.position_error.x = local_x_cm - (historic_position_base.x + inav.position_correction.x);
//			inav.position_error.y = local_y_cm - (historic_position_base.y + inav.position_correction.y);
			inav.position_error.z = z - (historic_position_base.z + inav.position_correction.z);
		}
	}
	inav.flag_baro_glitching = glitching_status;

}
/**
 * @brief update navigation velocity and position based on EXternal Position feedback if new available
 */
static void checkBaro(void)
{
	uint32_t now = millis();

	if( sens_baro.stamp > inav.baro_last)
	{
		float dt = (sens_baro.stamp - inav.baro_last)*0.001f;
		inav.baro_last_update = now;
		correctWithBaro(dt);
		inav.baro_last = sens_baro.stamp;
	}
	else
	{
		// if EXT POS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
		if (now - inav.baro_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		{
//			inav.position_error.x *= 0.9886f;
//			inav.position_error.y *= 0.9886f;
			inav.position_error.z *= 0.9886f;
		}
	}
}

static int isSonarGlitching(void)
{
	// calculate time since last sane gps reading in ms
	float sane_dt = (sens_sonar.stamp - inav.last_good_sonar_update) / 1000.0f;

	float distance_cm = sens_sonar.depth - inav.last_good_sonar;
//	debug("distance to last_good_lat is %.2f", distance_cm);

	int all_ok = 1;
//	q[2] = distance_cm;
		// all ok if within a given hardcoded radius

	if(isnan(sens_sonar.depth))
		all_ok = 0;

	if(isinf(sens_sonar.depth))
		all_ok = 0;

	if (fabs(distance_cm) > SONAR_RADIUS_CM)
	{
		all_ok = 0;
	}

	if(sens_sonar.depth == 0)
	{
		all_ok = 0;
	}

	if(all_ok == 1)
	{
		inav.last_good_sonar = sens_sonar.depth;
		inav.last_good_sonar_update = sens_sonar.stamp;
	}
	else
	{
		debug("SONAR glitched. previous sonar reading is %.2f; Current Sonar reading is %.2f", inav.last_good_sonar, sens_sonar.depth);
	}
	return (!all_ok);
}

static void correctWithSonar(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

	float z = sens_sonar.depth;

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	int glitching_status = isSonarGlitching();
	if(glitching_status)
	{
		// failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
		inav.position_error.z *= 0.89125f;
	}
	else
	{
		// if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
		// reset the inertial nav position and velocity to gps values
		if(inav.flag_sonar_glitching == 1)
		{
			debug("Z : sonar: %z pos base %.2f; pos_correction is %.2f; pos_error is %.2f",
							z, inav.position_base.z, inav.position_correction.z, inav.position_error.z);

			setAltitude(z);
			inav.position_error.z = 0;
		}
		else
		{
			Vector3f historic_position_base;

			if(inav.historic_z_property.is_full)
				historic_position_base.z = popQueue(inav.historic_z, &inav.historic_z_property);
			else
				historic_position_base.z = inav.position_base.z;

//			debug("position from sonar is %.2f; historic_position_base is %.2f; position_correction is %.2f",
//							z, inav.position_base.z, inav.position_correction.z);

			inav.position_error.z = z - (historic_position_base.z + inav.position_correction.z);
		}
	}
	inav.flag_sonar_glitching = glitching_status;
}


static void checkSonar(void)
{
	uint32_t now = millis();

	if( sens_sonar.stamp > inav.sonar_last)
	{
		float dt = (sens_sonar.stamp - inav.sonar_last)*0.001f;
		inav.sonar_last_update = now;
		correctWithSonar(dt);
		inav.sonar_last = sens_sonar.stamp;
	}
	else
	{
		// if EXT POS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
		if (now - inav.sonar_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		{
			inav.position_error.z *= 0.9886f;
		}
	}
}


void initializeGPSHome()
{
	bool flag_GPS_HOME_FOUND = 0;
	while(!flag_GPS_HOME_FOUND)
	{
		//wait till a location close to 50KM radius of IITK is found from GPS
		//TODO change this to using HDOP or something
		//right now not getting that data so can't do much
		debug("Waiting for GPS lock close to IITK. Current GPS lat is : %f; GPS long is : %f", sens_gps.lat*1e-7, sens_gps.lng*1e-7);
		if((fabs(sens_gps.lat*1e-7 - 26.5) + fabs(sens_gps.lng*1e-7 - 80.2)) < 1)
			flag_GPS_HOME_FOUND = 1;
		delay(100);
	}

	debug("Found GPS lock close to IITK. Current GPS lat is : %f; GPS long is : %f", sens_gps.lat*1e-7, sens_gps.lng*1e-7);
	ahrs.lat_home = sens_gps.lat;
	ahrs.lng_home = sens_gps.lng;
	inav.last_good_lat = sens_gps.lat;
	inav.last_good_lng = sens_gps.lng;
	inav.last_good_gps_update = sens_gps.stamp;

	//as soon as a proper GPS lock signal is obtained

	inav.lon_to_cm_scaling = cosf(ahrs.lat_home*1e-7f*DEG_TO_RAD)*LATLON_TO_CM;

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

void initializeAlt()
{
	debug("Initializing altitude");
	bool flag_EXT_POS_HOME_FOUND = 0;
	while(!flag_EXT_POS_HOME_FOUND)
	{
#if (USE_BARO_NOT_SONAR == 1)
		debug("Waiting for Baro to send data");
		if(sens_baro.stamp != 0)
			flag_EXT_POS_HOME_FOUND = 1;
#else
		debug("Waiting for SONAR to send data");
		if(sens_sonar.stamp != 0)
			flag_EXT_POS_HOME_FOUND = 1;
#endif
		delay(100);
	}

#if (USE_BARO_NOT_SONAR == 1)
	inav.position_base.z = sens_baro.position.z;
	inav.last_good_baro.z = sens_baro.position.z;
	inav.last_good_baro_update = sens_baro.stamp;
	inav.position_correction.z = 0.0f;
	inav.position.z = sens_baro.position.z;				//set initial altitude to whatever value is reached
	debug("Received Baro data. Altitude is %f", sens_baro.position.z);
#else
	inav.position_base.z = sens_sonar.depth;
	inav.last_good_baro.z = sens_sonar.depth;
	inav.last_good_baro_update = sens_sonar.stamp;
	inav.position_correction.z = 0.0f;
	inav.position.z = sens_sonar.depth;				//set initial altitude to whatever value is reached
	debug("Received SONAR data. Altitude is %f", sens_sonar.depth);
#endif

	inav.position_error.z = 0.0f;		//initial error is zero
	inav.velocity.z = 0.0f;				//set initialize altitude to 0

}

void initINAV()
{
	debug("Initializing the INAV portion");
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
	updateINAVGains();

	//update cv variables
	inav.cv_last = 0;
	inav.cv_last_update = 0;
	sens_cv.flag_active = 0;

	//update gps time variables
	inav.gps_last = 0;
	inav.gps_last_update = 0;

	//initialize IMU
	inav.last_good_imu = sens_imu.accel_calib;

#if(USE_GPS_NOT_CV == 1)
	initializeGPSHome();
#endif

	//TODO find out if initialization with some initial values and averagin is useful
	//acceleration_correction_hbf remaining :  can be initialized by considering some intial values OR maybe be left to converge
	initializeVector3fToZero(&inav.accel_correction_hbf);
	initializeAlt();

}

void resetINAV()
{
	debug("Resetting the Inertial Navigation code");
	// initialize the queues
	inav.historic_xy_counter = 0;
	inav.historic_z_counter = 0;
	inav.historic_x_property.size = AP_HISTORIC_XY_SIZE;
	inav.historic_y_property.size = AP_HISTORIC_XY_SIZE;
	inav.historic_z_property.size = AP_HISTORIC_Z_SIZE;
	resetQueue(inav.historic_x, &inav.historic_x_property);
	resetQueue(inav.historic_y, &inav.historic_y_property);
	resetQueue(inav.historic_z, &inav.historic_z_property);

	//update cv variables
	inav.cv_last = 0;
	inav.cv_last_update = 0;

	//update gps time variables
	inav.gps_last = 0;
	inav.gps_last_update = 0;

#if(USE_GPS_NOT_CV == 1)
	initializeGPSHome();
#endif

	//initialize IMU
	inav.last_good_imu = sens_imu.accel_calib;

	inav.sonar_last = 0;
	inav.sonar_last_update = 0;
	initializeAlt();

}

void updateINAVGains()
{
	debug("Previous inav gains k1_xy : %f, k2_xy : %f, k3_xy", inav.k1_xy, inav.k2_xy, inav.k3_xy);
	debug("Previous inav gains k1_z : %f, k2_z : %f, k3_z", inav.k1_xy, inav.k2_xy, inav.k3_xy);
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
	debug("Current inav gains k1_xy : %f, k2_xy : %f, k3_xy", inav.k1_xy, inav.k2_xy, inav.k3_xy);
	debug("Current inav gains k1_z : %f, k2_z : %f, k3_z", inav.k1_xy, inav.k2_xy, inav.k3_xy);
}

void updateINAV(uint32_t del_t)
{
	float dt = del_t * 0.001f;

//	debug("delta_time for INAV is %.2f", dt);

	if(dt > INERTIAL_NAV_DELTAT_MAX)
		return;

//	debug_vec[0] = dt;		//temporary variables for logging
	// check if new gps readings have arrived and use them to correct position estimates
#if (USE_GPS_NOT_CV == 1)
	checkGPS();
#else
	checkCV();
#endif

	// check if new readings have arrives from the external position sensor on the OBC
#if (USE_BARO_NOT_SONAR == 1)
	checkBaro();
#else
	checkSonar();
#endif

	Vector3f accel_ef;

	accel_ef.x = ahrs.accel_ef.x*100;	//acceleration in ahrs obtained is in mss and we are storing in cmss
	accel_ef.y = ahrs.accel_ef.y*100;	//acceleration in ahrs obtained is in mss and we are storing in cmss

	accel_ef.z = -(ahrs.accel_ef.z*100 + GRAVITY_CMSS); //converting acceleration from NED to NEU for proper calculation of altitude

	float accel_norm = normVec3f(accel_ef);

	//assuming body acceleration more than 10mss is not possible and that this is probably a glitch
	if(accel_norm > MAX_BODY_ACCEL)
		return;


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
//	debug_vec[1] = inav.position_error.z * inav.k2_z  * dt;
//	debug("velocity is %.2f: velocity correction is %.2f", inav.velocity.z, inav.position_error.z * inav.k2_z  * dt);

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

//	debug("position base %.2f; position correction is %.2f; position_error is %.2f",
//					inav.position_base.z, inav.position_correction.z, inav.position_error.z);

	// calculate new velocity
	inav.velocity.x += velocity_increase.x;
	inav.velocity.y += velocity_increase.y;
	inav.velocity.z += velocity_increase.z;

//	debug("velocity is %.2f, velocity increase is %.2f", inav.velocity.z, velocity_increase.z);

	pushToQueue(inav.position_base.z, inav.historic_z, &inav.historic_z_property);

	// store 3rd order estimate (i.e. horizontal position) for future use at 10hz
	inav.historic_xy_counter++;
	if( inav.historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS )
	{
		inav.historic_xy_counter = 0;
		pushToQueue(inav.position_base.x, inav.historic_x, &inav.historic_x_property);
//		printQueue(inav.historic_x, inav.historic_x_property);
		pushToQueue(inav.position_base.y, inav.historic_y, &inav.historic_y_property);
	}

}
