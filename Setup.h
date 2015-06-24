/*
 * Setup.h
 *
 *  Created on: 28-Aug-2014
 *      Author: nikhil
 */

#ifndef SETUP_H_
#define SETUP_H_

/**
 * @Warning DO NOT EDIT THIS FILE!
 * This file contain function related to System driver initialization
 */
#include <string.h>
#include "ch.h"
#include "hal.h"

extern SerialUSBDriver SDU1;

void StartTelemetry(void);

uint32_t millis(void);

void start_intercomm(void);

void start_blink(void);

void start_sys(void);

#endif /* SETUP_H_ */
