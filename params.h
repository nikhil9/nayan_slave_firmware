/*
 * params.h
 *
 *  Created on: 24-Oct-2015
 *      Author: atulya
 */

#include "main.h"
#include "mavlink_types.h"

#ifndef PARAMS_H_
#define PARAMS_H_

extern const char sysid_thismav[16];
extern const char sysid_sw_type[16];
extern const char sysid_mygcs[16];

extern const char inav_tc_xy[16];
extern const char inav_tc_z[16];

extern const char pos_xy_p[16];
extern const char vel_xy_p[16];
extern const char vel_xy_i[16];
extern const char vel_xy_imax[16];

extern const char pos_z_p[16];
extern const char vel_z_p[16];
extern const char accel_z_p[16];
extern const char accel_z_i[16];
extern const char accel_z_d[16];
extern const char accel_z_imax[16];
extern const char accel_z_filt_hz[16];

extern const char sysid_thismav_index;
extern const char sysid_sw_type_index;
extern const char sysid_mygcs_index;

extern const uint8_t inav_tc_xy_index;
extern const uint8_t inav_tc_z_index;

extern const uint8_t pos_xy_p_index;
extern const uint8_t vel_xy_p_index;
extern const uint8_t vel_xy_i_index;
extern const uint8_t vel_xy_imax_index;

extern const uint8_t pos_z_p_index;
extern const uint8_t vel_z_p_index;
extern const uint8_t accel_z_p_index;
extern const uint8_t accel_z_i_index;
extern const uint8_t accel_z_d_index;
extern const uint8_t accel_z_imax_index;
extern const uint8_t accel_z_filt_hz_index;

#define PARAM_COUNT 16				//total number of parameters to be sent

//IF YOU WANT TO DECLARE FWupdateParamMavLink and FWparamQSend in the .c file then you will have to make comm_ch_send static inside mavlink_helpers.h
//currently only one instance of the mavlink _helpers is required in the odroid_comm.c file
//the functions could also have been declared inside the odroid_comm_c but if the length of parameters increases then the code can get cluttered

void FWupdateParamMavLink(char _name[17], float _paramValue){

	if(strcmp(_name, sysid_thismav) == 0){
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 sysid_thismav,
				 1, 			//param value
				 MAVLINK_TYPE_UINT8_T,	//param type
				 PARAM_COUNT,
				 sysid_thismav_index);
	  }

	if(strcmp(_name, sysid_sw_type) == 0){
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 sysid_sw_type,
				 10, 			//param value
				 MAVLINK_TYPE_UINT8_T,	//param type
				 PARAM_COUNT,
				 sysid_sw_type_index);
		  }
	if(strcmp(_name, sysid_mygcs) == 0){
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 sysid_mygcs,
				 255, 			//param value
				 MAVLINK_TYPE_UINT8_T,	//param type
				 PARAM_COUNT,
				 sysid_mygcs_index);
		  }

	if(strcmp(_name, inav_tc_xy) == 0){
		 inav.time_constant_xy = _paramValue;
		 updateINAVGains();
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 inav_tc_xy,
				 inav.time_constant_xy, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 inav_tc_xy_index);
	  }

	if(strcmp(_name, inav_tc_z) == 0){
		 inav.time_constant_z = _paramValue;
		 updateINAVGains();
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 inav_tc_z,
				 inav.time_constant_z, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 inav_tc_z_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, pos_xy_p) == 0){
		 pos_control._p_pos_xy.kP = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 pos_xy_p,
				 pos_control._p_pos_xy.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 pos_xy_p_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, vel_xy_p) == 0){
		pos_control._pi_vel_xy.kP = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_p,
				 pos_control._pi_vel_xy.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 vel_xy_p_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, vel_xy_i) == 0){
		pos_control._pi_vel_xy.kI = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_i,
				 pos_control._pi_vel_xy.kI, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 vel_xy_i_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, vel_xy_imax) == 0){
		pos_control._pi_vel_xy.Imax = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_imax,
				 pos_control._pi_vel_xy.Imax, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 vel_xy_imax_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, pos_z_p) == 0){
		pos_control._p_pos_z.kP = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 pos_z_p,
				 pos_control._p_pos_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 pos_z_p_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, vel_z_p) == 0){
		pos_control._p_vel_z.kP = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_z_p,
				 pos_control._p_vel_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 vel_z_p_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, accel_z_p) == 0){
		pos_control._pid_accel_z.kP = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_p,
				 pos_control._pid_accel_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 accel_z_p_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, accel_z_i) == 0){
		pos_control._pid_accel_z.kI = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_i,
				 pos_control._pid_accel_z.kI, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 accel_z_i_index);		//_queued_parameter_index
	  }
	if(strcmp(_name, accel_z_d) == 0){
		pos_control._pid_accel_z.kD = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_d,
				 pos_control._pid_accel_z.kD, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 accel_z_d_index);		//_queued_parameter_index
	  }
	if(strcmp(_name, accel_z_imax) == 0){
		pos_control._pid_accel_z.Imax = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_imax,
				 pos_control._pid_accel_z.Imax, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 accel_z_imax_index);		//_queued_parameter_index
	  }

	if(strcmp(_name, accel_z_filt_hz) == 0){
		pos_control._pid_accel_z.filt_hz = _paramValue;
		// writeEEPROM(param_set.param_value, rateX_P_Index);
		 palSetPad(GPIOC, 12);
		 chThdSleepMilliseconds(2);
		 palClearPad(GPIOC, 12);

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_filt_hz,
				 pos_control._pid_accel_z.filt_hz, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,
				 accel_z_filt_hz_index);		//_queued_parameter_index
	  }

}


void FWparamQSend(void){

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 inav_tc_xy,
				 inav.time_constant_xy, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 inav_tc_xy_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 inav_tc_z,
				 inav.time_constant_z, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 inav_tc_z_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 pos_xy_p,
				 pos_control._p_pos_xy.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 pos_xy_p_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_p,
				 pos_control._pi_vel_xy.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 vel_xy_p_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_i,
				 pos_control._pi_vel_xy.kI, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 vel_xy_i_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_xy_imax,
				 pos_control._pi_vel_xy.Imax, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 vel_xy_imax_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 pos_z_p,
				 pos_control._p_pos_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 pos_z_p_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 vel_z_p,
				 pos_control._p_vel_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 vel_z_p_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_p,
				 pos_control._pid_accel_z.kP, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 accel_z_p_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_i,
				 pos_control._pid_accel_z.kI, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 accel_z_i_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_d,
				 pos_control._pid_accel_z.kD, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 accel_z_d_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_imax,
				 pos_control._pid_accel_z.Imax, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 accel_z_imax_index);		//_queued_parameter_index

		 mavlink_msg_param_value_send(
				 MAVLINK_COMM_0,
				 accel_z_filt_hz,
				 pos_control._pid_accel_z.filt_hz, 			//param value
				 MAVLINK_TYPE_FLOAT,	//param type
				 PARAM_COUNT,		//_queued_parameter_count
				 accel_z_filt_hz_index);		//_queued_parameter_index

}



//void updatePIDFromEEPROM(void);	TODO FUTURE SCOPE
//void writeDefaultParam(void);		TODO FUTURE SCOPE

#endif /* PARAMS_H_ */
