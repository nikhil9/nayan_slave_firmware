/*
 * main.h
 *
 *  Created on: Aug 2, 2014
 *      Author: nikhil
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"
#include "hal.h"
#include "Setup.h"
#include "intercomm.h"

typedef struct{
	float x;
	float y;
	float z;

}vector_3f;


extern vector_3f accel;

extern vector_3f gyro;

extern vector_3f attitude;

extern vector_3f position;

extern vector_3f velocity;

extern uint16_t rc_in[7];

#endif /* MAIN_H_ */
