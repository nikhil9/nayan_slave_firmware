/*
 * odroid_comm.c
 *
 *  Created on: 26-Apr-2015
 *      Author: nikhil
 */

#ifndef ODROID_COMM_C_
#define ODROID_COMM_C_

/**
 * @Warning	EDIT THIS FILE WHEN YOU ARE 200% SURE OF WHAT YOU ARE DOING!
 * @note	For more information on Mavlink please visit http://qgroundcontrol.org/mavlink/start
 * @brief	This file contain function related to intercommunication between Slave Processor and On-Board Computer
 */


#include "ch.h"
#include "hal.h"
#include "Setup.h"

//=================ADDITION FROM MAVLINK====================

#include "mavlink_types.h"

#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#endif

static mavlink_system_t mavlink_system;

static void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	sdPut(&SDU1, ch );
    }
}

#include "OS_PORT/ext/mavlink/v1.0/common/mavlink.h"

//==========================================================

#include "intercomm.h"
#include "main.h"


const float MILLIG_TO_MS2 = 9.80665f / 1000.0f;
const float MS2_TO_MILLIG = 1000.0f/9.80665f;
const float RAD_TO_MILLIRAD = 1000.0f;

//Sends Hearthbeat to odroid
static void send_heart_beat(mavlink_channel_t chan){

	mavlink_msg_heartbeat_send(chan,
			2,
			0,
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		MAV_MODE_FLAG_STABILIZE_ENABLED,
		MAV_STATE_ACTIVE);
}

//Sends imu data to odroid
static void send_scaled_imu(mavlink_channel_t chan){

	mavlink_msg_scaled_imu_send(chan,
			  millis(),
			  (int16_t)(sens_imu.accel.x*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.accel.y*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.accel.z*MS2_TO_MILLIG),
			  (int16_t)(sens_imu.gyro.x*RAD_TO_MILLIRAD),
			  (int16_t)(sens_imu.gyro.y*RAD_TO_MILLIRAD),
			  (int16_t)(sens_imu.gyro.z*RAD_TO_MILLIRAD),
			  (int16_t)0,
			  (int16_t)0,
			  (int16_t)0);
}

//Sends attitude data to odroid
static void send_attitude(mavlink_channel_t chan){

	mavlink_msg_attitude_send(
			chan,
			millis(),
			sens_imu.attitude.x,
			sens_imu.attitude.y,
			sens_imu.attitude.z,
			sens_imu.gyro.x,
			sens_imu.gyro.y,
			sens_imu.gyro.z);
}

//Sends gps-baro position data to odroid
static void send_gps(mavlink_channel_t chan){

	mavlink_msg_global_position_int_send(
			chan,
			millis(),
			sens_pos.lat,						//sending out raw gps data as received from LLP
			sens_pos.lng,
			sens_pos.alt*CM_TO_MM,
			sens_pos.alt*CM_TO_MM,
			sens_pos.vel.x,
			sens_pos.vel.y,
			sens_pos.vel.z,
			(int16_t)(sens_imu.attitude.z*5729.57));
}

//Sends rc data to odroid
static void send_rc_in(mavlink_channel_t chan){
    mavlink_msg_rc_channels_raw_send(
    	chan,
        millis(),
        0, // port
        rc_in[0],								//sending out raw RC input data as received from LLP
		rc_in[1],
		rc_in[2],
		rc_in[3],
		rc_in[4],
		rc_in[5],
		rc_in[6],
		0,
        -1);
}


static WORKING_AREA(mavlinkSendThread, 8192);
static msg_t mavlinkSend(void *arg) {

  (void)arg;
  chRegSetThreadName("mavlinkSend");

  uint16_t hbt_cnt = 0;

  uint16_t gps_cnt = 0;

  uint16_t rc_cnt = 0;

  while (TRUE) {

	  //Sending out Heartbeat @ 2Hz
	  if(hbt_cnt > 100){
		  send_heart_beat(MAVLINK_COMM_0);
		  hbt_cnt = 0;
	  }

	  //sending out imu data @ 200Hz
	  send_scaled_imu(MAVLINK_COMM_0);

	  send_attitude(MAVLINK_COMM_0);

	  //sending out GPS data @ 50Hz
	  if(gps_cnt > 40){
		  send_gps(MAVLINK_COMM_0);
		  gps_cnt = 0;
	  }

	  //sending out RC data @ 50Hz
	  if(rc_cnt > 4)
	  {
		  send_rc_in(MAVLINK_COMM_0);
		  rc_cnt = 0;
	  }

	  chThdSleep(US2ST(4500));
	  hbt_cnt++;
	  gps_cnt++;
	  rc_cnt++;

  }
  return 0;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
uint16_t comm_get_available(mavlink_channel_t chan){
	int16_t bytes = 0;
	   switch(chan) {
		case MAVLINK_COMM_0:
			bytes = chQSpaceI(&(&SDU1)->iqueue);
			break;
		default:
			break;
	   }
	   return bytes;
}

/**
 * @Warning DO NOT EDIT THIS FUNCTION!
 */
uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		data = sdGetTimeout(&SDU1, 2);
		break;

	default:
		break;
	}
    return data;
}

//Function is necessory to work properly with mavros. If you define your param, then you can send it via this function
void send_params(mavlink_channel_t chan){
	mavlink_msg_param_value_send(
					 chan,
					 "SYSID_THISMAV",
					 1,
					 MAVLINK_TYPE_UINT8_T,
					 0,
					 3);

	mavlink_msg_param_value_send(
					 chan,
					 "SYSID_SW_TYPE",
					 10,
					 MAVLINK_TYPE_UINT8_T,
					 1,
					 3);

	mavlink_msg_param_value_send(
					 chan,
					 "SYSID_MYGCS",
					 255,
					 MAVLINK_TYPE_UINT8_T,
					 2,
					 3);
}

//Function to handle incoming data from mavro. If you wish to add custom
//message, then add an id for corresponding message within switch to handle it
void handleMessage(mavlink_message_t* msg, mavlink_channel_t chan)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
    	send_params(chan);
    	break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
    	mavlink_msg_mission_count_send(chan, 1, 0, 0);
    	break;
    }

    default:
    	break;

    }
}

//Function to receive and update data from mavros
void odroid_update(void )
{

    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;


    uint16_t nbytes = comm_get_available(MAVLINK_COMM_0);

    uint16_t i = 0;

    for (i = 0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(MAVLINK_COMM_0);

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

            handleMessage(&msg, MAVLINK_COMM_0);
        }
    }
}

static WORKING_AREA(mavlinkReceiveThread, 8192);
static msg_t mavlinkReceive(void *arg) {

  (void)arg;
  chRegSetThreadName("mavlinkReceive");

  while (TRUE) {

	  odroid_update();

	  delay(10);
  }
  return 0;
}

//Function to initiate odroid communication
void odroid_comm_init(void){

	mavlink_system.sysid = 1;

	chThdCreateStatic(mavlinkReceiveThread, sizeof(mavlinkReceiveThread), NORMALPRIO, mavlinkReceive, NULL);
	chThdCreateStatic(mavlinkSendThread, sizeof(mavlinkSendThread), NORMALPRIO, mavlinkSend, NULL);

}


#endif /* ODROID_COMM_C_ */
