/*
 * odroid_comm.h
 *
 *  Created on: 26-Apr-2015
 *      Author: nikhil
 */

#ifndef ODROID_COMM_H_
#define ODROID_COMM_H_


/**
 * @Warning DO NOT EDIT THIS FILE!
 * This file contain function related to intercommunication between Slave Processor and On-Board Computer
 */

void odroid_comm_init(void);

void send_params(void);

void odroid_debug(uint8_t _index, float _value);

#endif /* ODROID_COMM_H_ */
