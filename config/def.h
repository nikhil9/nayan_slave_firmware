/*
 * defins.h
 *
 *  Created on: Mar 24, 2014
 *      Author: nikhil
 */

#ifndef CONFIG_DEF_H_
#define CONFIG_DEF_H_

#include"hal.h"
#include"ch.h"
#include "Setup.h"

#define LOG				TRUE

#define TELE_PORT		SD2

#define TELE2_PORT		SD4

#define GPS_PORT		SD4

#define USE_EKF			TRUE

//0 -> OFF
//1 -> Debug
//2 -> Tele

#define DUAL_TELE		0

#define EX_DBG			1

#define I2C_MPU 		I2CD1

#define I2C_AK			I2CD3

#define I2C_MPL			I2CD3

#define I2C_EE			I2CD2

#define I2C_ST			I2CD2

#define I2C_HMC			I2CD3

#define I2C_OPT			I2CD2

#define SBUS_PORT		SD1

#define I2C_MS			I2CD2

#endif /* CONFIG_DEF_H_ */

