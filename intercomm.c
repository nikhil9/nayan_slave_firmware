/*
 * intercomm.cpp
 *
 *  Created on: 16-Apr-2015
 *      Author: nikhil
 */

#ifndef INTERCOMM_CPP_
#define INTERCOMM_CPP_

/**
 * @Warning DO NOT EDIT THIS FILE!
 * This file contain function related to intercommunication between Master and Slave processor
 */


#include "Setup.h"
#include "intercomm.h"
#include "config/def.h"
#include "main.h"

#define MAX_ACCEL_MPU6050 	80.0f						//+- 8g
#define MAX_GYRO_MPU6050 	2000.0f/(2*M_PI_F)			//+- 2000 degps

ic_imu_u ic_imu_data;

ic_rc_or_u ic_rc_or_data;

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

void spi_exchange_data(SPIDriver *spip, uint8_t *tx, uint8_t *rx, size_t size) {

	spiAcquireBus(spip);
	spiSelect(spip);
	spiExchange(spip, size, tx, rx);
	spiUnselect(spip);
	spiReleaseBus(spip);
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
bool_t check_ahrs_sanity(float _a){

	if(_a > 5 || _a < -5){
		return FALSE;
	}
	return TRUE;

}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
bool_t check_acc_sanity(float _a){

	if(_a > 16 || _a < -16){
		return FALSE;
	}
	return TRUE;

}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

bool_t isAccelSane(void)
{
	if(isnan(ic_imu_data.ic_imu.ax)||isnan(ic_imu_data.ic_imu.ay)||isnan(ic_imu_data.ic_imu.az))
		return FALSE;

	if(isinf(ic_imu_data.ic_imu.ax)||isinf(ic_imu_data.ic_imu.ay)||isinf(ic_imu_data.ic_imu.az))
		return FALSE;

	//MAXIMUM measurement possible is +- 8g
	if((fabs(ic_imu_data.ic_imu.ax)>MAX_ACCEL_MPU6050) ||(fabs(ic_imu_data.ic_imu.ay)>MAX_ACCEL_MPU6050) ||
			(fabs(ic_imu_data.ic_imu.az)>MAX_ACCEL_MPU6050))
		return FALSE;

	return TRUE;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

bool_t isGyroSane(void)
{

	if(isnan(ic_imu_data.ic_imu.ax)||isnan(ic_imu_data.ic_imu.ay)||isnan(ic_imu_data.ic_imu.az))
		return FALSE;

	if(isinf(ic_imu_data.ic_imu.ax)||isinf(ic_imu_data.ic_imu.ay)||isinf(ic_imu_data.ic_imu.az))
		return FALSE;

	//MAXIMUM measurement possible is +- 2000 degps
	if((fabs(ic_imu_data.ic_imu.gx)>MAX_GYRO_MPU6050) ||(fabs(ic_imu_data.ic_imu.gy)>MAX_GYRO_MPU6050) ||
			(fabs(ic_imu_data.ic_imu.gz)>MAX_GYRO_MPU6050))
		return FALSE;

	return TRUE;

}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

bool_t isAttitudeSane(void)
{

	if(isnan(ic_imu_data.ic_imu.roll)||isnan(ic_imu_data.ic_imu.pitch)||isnan(ic_imu_data.ic_imu.yaw))
		return FALSE;

	if(isinf(ic_imu_data.ic_imu.roll)||isinf(ic_imu_data.ic_imu.pitch)||isinf(ic_imu_data.ic_imu.yaw))
		return FALSE;

	//assuming that the maximum attitude angles is 10 rad
	if((fabs(ic_imu_data.ic_imu.roll)>10)||(fabs(ic_imu_data.ic_imu.pitch)>10)||(fabs(ic_imu_data.ic_imu.yaw)>10))
		return FALSE;

	return TRUE;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

bool_t isRCInputSane(void)
{

	if(isnan(ic_imu_data.ic_imu.rc_in_1) || isnan(ic_imu_data.ic_imu.rc_in_2) || isnan(ic_imu_data.ic_imu.rc_in_3) ||
			isnan(ic_imu_data.ic_imu.rc_in_4) || isnan(ic_imu_data.ic_imu.rc_in_5) || isnan(ic_imu_data.ic_imu.rc_in_6) ||
			isnan(ic_imu_data.ic_imu.rc_in_7))
		return FALSE;

	if(isinf(ic_imu_data.ic_imu.rc_in_1) || isinf(ic_imu_data.ic_imu.rc_in_2) || isinf(ic_imu_data.ic_imu.rc_in_3) ||
			isinf(ic_imu_data.ic_imu.rc_in_4) || isinf(ic_imu_data.ic_imu.rc_in_5) || isinf(ic_imu_data.ic_imu.rc_in_6) ||
			isinf(ic_imu_data.ic_imu.rc_in_7))
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_1 < 700 || ic_imu_data.ic_imu.rc_in_1 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_2 < 700 || ic_imu_data.ic_imu.rc_in_2 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_3 < 700 || ic_imu_data.ic_imu.rc_in_3 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_4 < 700 || ic_imu_data.ic_imu.rc_in_4 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_5 < 700 || ic_imu_data.ic_imu.rc_in_5 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_6 < 700 || ic_imu_data.ic_imu.rc_in_6 > 2300)
		return FALSE;

	if(ic_imu_data.ic_imu.rc_in_7 < 700 || ic_imu_data.ic_imu.rc_in_7 > 2300)
		return FALSE;

	return TRUE;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */

bool_t isGPSBaroSane(void)
{

	if(isinf(ic_imu_data.ic_imu.lat) || isinf(ic_imu_data.ic_imu.lng) || isinf(ic_imu_data.ic_imu.alt))
		return FALSE;

	if(isnan(ic_imu_data.ic_imu.lat) || isnan(ic_imu_data.ic_imu.lng) || isnan(ic_imu_data.ic_imu.alt))
		return FALSE;

	//assuming maximum flight altitude is 100m
	if(fabs(ic_imu_data.ic_imu.alt) > 10000)
		return FALSE;

	return TRUE;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION EVER!
 */
void update_ic_data(void){

	uint32_t stamp = millis();

	if(isAccelSane() == TRUE && isGyroSane() == TRUE && isAttitudeSane() == TRUE)
	{
		sens_imu.stamp = stamp;

		//acceleration outputs in m/s2 in NED body frame
		sens_imu.accel.x = ic_imu_data.ic_imu.ax;
		sens_imu.accel.y = ic_imu_data.ic_imu.ay;
		sens_imu.accel.z = ic_imu_data.ic_imu.az;

		//angular velocity outputs in NED body frame in rad/s
		sens_imu.gyro.x = ic_imu_data.ic_imu.gx;
		sens_imu.gyro.y = ic_imu_data.ic_imu.gy;
		sens_imu.gyro.z = ic_imu_data.ic_imu.gz;

		//attitude Roll, Pitch and Yaw as calculated about NED frame in rad
		sens_imu.attitude.x = ic_imu_data.ic_imu.roll;
		sens_imu.attitude.y = ic_imu_data.ic_imu.pitch;
		sens_imu.attitude.z = ic_imu_data.ic_imu.yaw;
	}

	if(isGPSBaroSane() == TRUE)
	{
		//GPS latitude(in degrees)*1e7; longitude(in degrees)*1e7 ; altitude(in cm)
		Vector3f _position_loc;
		_position_loc.x = ic_imu_data.ic_imu.lat;
		_position_loc.y = ic_imu_data.ic_imu.lng;
		_position_loc.z = ic_imu_data.ic_imu.alt;

		if(_position_loc.x != sens_pos.lat || _position_loc.y != sens_pos.lng || _position_loc.z != sens_pos.alt)
		{
			sens_pos.stamp = stamp;
			sens_pos.lat = (int32_t)(_position_loc.x);
			sens_pos.lng = (int32_t)(_position_loc.y);
			sens_pos.alt = _position_loc.z;

		}
	}

	//velocity as processed by the LLP
	sens_pos.vel.x = ic_imu_data.ic_imu.vx;
	sens_pos.vel.y = ic_imu_data.ic_imu.vy;
	sens_pos.vel.z = ic_imu_data.ic_imu.vz;

	if(isRCInputSane() == TRUE)
	{
		//RC channel inputs as obtained from transmitter corresponding to channels 1 to 7
		rc_in[0] = ic_imu_data.ic_imu.rc_in_1;
		rc_in[1] = ic_imu_data.ic_imu.rc_in_2;
		rc_in[2] = ic_imu_data.ic_imu.rc_in_3;
		rc_in[3] = ic_imu_data.ic_imu.rc_in_4;
		rc_in[4] = ic_imu_data.ic_imu.rc_in_5;
		rc_in[5] = ic_imu_data.ic_imu.rc_in_6;
		rc_in[6] = ic_imu_data.ic_imu.rc_in_7;
	}


	if(dmc){
		ic_rc_or_data.ic_rc.rc7 = 1500;
	}else{
		ic_rc_or_data.ic_rc.rc7 = 1000;
	}
	ic_rc_or_data.ic_rc.rc1 = control_command[0];
	ic_rc_or_data.ic_rc.rc2 = control_command[1];
	ic_rc_or_data.ic_rc.rc3 = control_command[2];
	ic_rc_or_data.ic_rc.rc4 = control_command[3];
}

#endif /* INTERCOMM_CPP_ */
