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
	int all_ok = 0;

	//assuming that a sane accelerometer is at least 1mss
	if(normVec3f(sens_imu.accel_calib) > 1)
		all_ok = 1;

	if(all_ok == 1 )
		inav.last_good_imu_update = sens_imu.stamp;

	return (!all_ok);
}

void updateAHRS(void)
{
	if(isIMUGlitching())
		return ;

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
// calculate time since last sane gps reading in ms
	float sane_dt = (sens_gps.stamp - inav.last_good_gps_update) / 1000.0f;

	float dlat = sens_gps.lat - inav.last_good_lat;
	float dlong = sens_gps.lng - inav.last_good_lng;

	float distance_cm = sqrt(dlat*dlat + dlong*dlong)*LATLON_TO_CM;
//	debug("distance to last_good_lat is %f", distance_cm);

	int all_ok = 0;

	    // all ok if within a given hardcoded radius
	if (distance_cm <= GPS_RADIUS_CM)
	{
		all_ok = 1;
	}
	//TODO complete the accel_based distance
//	else
//	{
//		// or if within the maximum distance we could have moved based on our acceleration
//		accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
//		all_ok = (distance_cm <= accel_based_distance);
//	}

	if(all_ok == 1)
	{
		inav.last_good_lat = sens_gps.lat;
		inav.last_good_lng = sens_gps.lng;
		inav.last_good_gps_update = sens_gps.stamp;
	}
	return (!all_ok);
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
 * @brief correct the inav with new gps updates
 */
static void correctWithGPS(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

	x_cm = (sens_gps.lat - ahrs.lat_home) * LATLON_TO_CM;
	y_cm = (sens_gps.lng - ahrs.lng_home) * inav.lon_to_cm_scaling;

//	debug("GPS lat is %d; GPS lng is %d;", sens_gps.lat, sens_gps.lng);
//	debug("GPS x is %f; GPS y is %f; deltat is %f",x,y,dt);

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
			debug("position base %f; position correction is %f; position_error is %f",
							inav.position_base.x, inav.position_correction.x, inav.position_error.x);
			setPositionXY(x_cm,y_cm);
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

//			debug("position from gps is %f; historic_position_base is %f; position_correction is %f",
//							x_cm, inav.position_base.x, inav.position_correction.x);

			inav.position_error.x = x_cm - (historic_position_base.x + inav.position_correction.x);
			inav.position_error.y = y_cm - (historic_position_base.y + inav.position_correction.y);
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
static int isExtPosGlitching(void)
{
	// calculate time since last sane gps reading in ms
	float sane_dt = (sens_gps.stamp - inav.last_good_gps_update) / 1000.0f;

	float distance_cm = sens_ext_pos.position.z - inav.last_good_ext_pos.z;
	debug("distance to last_good_lat is %f", distance_cm);

	int all_ok = 1;

		// all ok if within a given hardcoded radius
	if (fabs(distance_cm) > EXT_POS_RADIUS_CM)
	{
		all_ok = 0;
	}

	if(sens_ext_pos.position.z == 0)
	{
		all_ok = 0;
	}

		//TODO UNNECESSARY complete the accel_based distance
	//	else
	//	{
	//		// or if within the maximum distance we could have moved based on our acceleration
	//		accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
	//		all_ok = (distance_cm <= accel_based_distance);
	//	}

	if(all_ok == 1)
	{
		inav.last_good_ext_pos.z = sens_ext_pos.position.z;
		inav.last_good_ext_pos_update = sens_ext_pos.stamp;
	}
	return (!all_ok);
}

/**
 * @brief correct the inav with updates from the external navigation sensor(vision/px4flow)
 */
static void correctWithExtPos(float dt)
{
	//discard samples if dt is too large
	if(dt > 1.0f || dt <= 0.0f)
		return;

//	x_cm = sens_ext_pos.position.x*100;
//	y_cm = sens_ext_pos.position.y*100;
	float z = sens_ext_pos.position.z;

//	debug("GPS lat is %d; GPS lng is %d;", sens_gps.lat, sens_gps.lng);
//	debug("GPS x is %f; GPS y is %f; deltat is %f",x,y,dt);

	// sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
	int glitching_status = isExtPosGlitching();
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
		if(inav.flag_ext_pos_glitching == 1)
		{
			debug("Z : ext_pos: %z pos base %f; pos_correction is %f; pos_error is %f",
							z, inav.position_base.z, inav.position_correction.z, inav.position_error.z);

//			setPositionXY(x_cm,y_cm);
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

			debug("position from ext_pos is %f; historic_position_base is %f; position_correction is %f",
							z, inav.position_base.z, inav.position_correction.z);

//			inav.position_error.x = x_cm - (historic_position_base.x + inav.position_correction.x);
//			inav.position_error.y = y_cm - (historic_position_base.y + inav.position_correction.y);
			inav.position_error.z = z - (historic_position_base.z + inav.position_correction.z);
		}
	}
	inav.flag_ext_pos_glitching = glitching_status;

}

/**
 * @brief update navigation velocity and position based on EXternal Position feedback if new available
 */
static void checkExtPos(void)
{
	uint32_t now = millis();

	if( sens_ext_pos.stamp > inav.ext_pos_last)
	{
		float dt = (sens_ext_pos.stamp - inav.ext_pos_last)*0.001f;
		inav.ext_pos_last_update = now;
		correctWithExtPos(dt);
		inav.ext_pos_last = sens_ext_pos.stamp;
	}
	else
	{
		// if EXT POS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
		if (now - inav.ext_pos_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS)
		{
//			inav.position_error.x *= 0.9886f;
//			inav.position_error.y *= 0.9886f;
			inav.position_error.z *= 0.9886f;
		}
	}
}


void initializeHome()
{
	bool flag_GPS_HOME_FOUND = 0;
	while(!flag_GPS_HOME_FOUND)
	{
		//wait till a location close to 50KM radius of IITK is found from GPS
		//TODO change this to using HDOP or something
		//right now not getting that data so can't do much
		debug("GPS lat is : %d; GPS long is : %d", sens_gps.lat, sens_gps.lng);
		if((fabs(sens_gps.lat*1e-7 - 26.5) + fabs(sens_gps.lng*1e-7 - 80.2)) < 1)
			flag_GPS_HOME_FOUND = 1;
		delay(100);
	}

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
	bool flag_EXT_POS_HOME_FOUND = 0;
	while(!flag_EXT_POS_HOME_FOUND)
	{
		if(sens_ext_pos.stamp != 0)
			flag_EXT_POS_HOME_FOUND = 1;
		delay(100);
	}

	inav.position_base.z = sens_ext_pos.position.z;
	inav.last_good_ext_pos.z = sens_ext_pos.position.z;
	inav.last_good_ext_pos_update = sens_ext_pos.stamp;
	inav.position_correction.z = 0.0f;
	inav.position.z = sens_ext_pos.position.z;				//set initial altitude to whatever value is reached

	inav.position_error.z = 0.0f;		//initial error is zero
	inav.velocity.z = 0.0f;				//set initialize altitude to 0

}

void initINAV()
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
	updateINAVGains();

	//update gps time variables
	inav.gps_last = 0;
	inav.gps_last_update = 0;

	initializeHome();
	//TODO UNNECESSARY initialize other variables
	//acceleration_correction_hbf remaining :  can be initialized by considering some intial values OR maybe be left to converge
	initializeAlt();

}

void updateINAVGains()
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

//	debug("delta_time for INAV is %f", dt);

	if(dt > INERTIAL_NAV_DELTAT_MAX)
		return;

	// check if new gps readings have arrived and use them to correct position estimates
	checkGPS();

	// check if new readings have arrives from the external position sensor on the OBC
	checkExtPos();

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
//	debug("velocity is %f: velocity correction is %f", inav.velocity.z, inav.position_error.z * inav.k2_z  * dt);

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

//	debug("position base %f; position correction is %f; position_error is %f",
//					inav.position_base.z, inav.position_correction.z, inav.position_error.z);

	// calculate new velocity
	inav.velocity.x += velocity_increase.x;
	inav.velocity.y += velocity_increase.y;
	inav.velocity.z += velocity_increase.z;

//	debug("velocity is %f, velocity increase is %f", inav.velocity.z, velocity_increase.z);

	//TODO CHECK the working
	// store 3rd order estimate (i.e. estimated vertical position) for future use
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
