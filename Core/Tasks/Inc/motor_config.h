/*
 * motor_config.h
 *
 *  Created on: 21 Dec 2021
 *      Author: wx
 */

#ifndef TASKS_INC_MOTOR_CONFIG_H_
#define TASKS_INC_MOTOR_CONFIG_H_


void config_motors();
void motor_calib_task(void* argument);


#define GM6020_MAX_OUTPUT 	20000
#define GM6020_MAX_RPM		400

#define M2006_MAX_RPM		15000
#define M2006_MAX_OUTPUT 	16384
#define M2006_GEARBOX_RATIO	36

#define M3508_MAX_OUTPUT 	16384
#define M3508_MAX_RPM		9000
#define M3508_GEARBOX_RATIO	19.2


#define	TYPE_GM6020 		1
#define	TYPE_M2006 			2
#define	TYPE_M3508 			3
#define	TYPE_M3508_NGEARBOX 4
#define	TYPE_M3508_STEPS 	5
#define	TYPE_M2006_STEPS 	6
#define	TYPE_M2006_ANGLE 	7
#define	TYPE_M3508_ANGLE 	8
#define	TYPE_GM6020_720		9

#endif /* TASKS_INC_MOTOR_CONFIG_H_ */
