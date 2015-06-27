/*
 * intercomm.h
 *
 *  Created on: 16-Apr-2015
 *      Author: nikhil
 */

#ifndef INTERCOMM_H_
#define INTERCOMM_H_

/**
 * @Warning DO NOT EDIT THIS FILE!
 * This file contain function related to intercommunication between Master and Slave processor
 */


#include "ch.h"
#include "hal.h"

#define IC_RC_OR_H	0x1E
#define IC_IMU_H	0x2E

typedef struct {
	float roll;
	float pitch;
	float yaw;

	float gx;
	float gy;
	float gz;

	float ax;
	float ay;
	float az;

	float vx;
	float vy;
	float vz;

	float lat;
	float lng;
	float alt;

	uint16_t rc_in_1;
	uint16_t rc_in_2;
	uint16_t rc_in_3;
	uint16_t rc_in_4;
	uint16_t rc_in_5;
	uint16_t rc_in_6;
	uint16_t rc_in_7;

}ic_imu_t;

typedef union {
	ic_imu_t ic_imu;
	uint8_t raw[74];
}ic_imu_u;

extern ic_imu_u ic_imu_data;

typedef struct {
	int16_t rc1;
	int16_t rc2;
	int16_t rc3;
	int16_t rc4;
	int16_t rc5;
	int16_t rc6;
	int16_t rc7;
}ic_rc_or_t;

typedef union {
	ic_rc_or_t ic_rc;
	uint8_t raw[14];
}ic_rc_or_u;

extern ic_rc_or_u ic_rc_or_data;

void spi_exchange_data(SPIDriver *spip, uint8_t *tx, uint8_t *rx, size_t size);

void spi_send_data(SPIDriver *spip, uint8_t *tx, size_t size);

void spi_receive_data(SPIDriver *spip, uint8_t *rx, size_t size);

void send_ic_rc_or(int16_t rc1,
		int16_t rc2,
		int16_t rc3,
		int16_t rc4,
		int16_t rc5,
		int16_t rc6,
		int16_t rc7);

void get_ic_imu(void);

bool_t check_ahrs_sanity(float _a);

void update_ic_data(void);

bool_t check_acc_sanity(float _a);

#endif /* INTERCOMM_H_ */