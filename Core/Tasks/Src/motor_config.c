/*
 * motor_config.c
 *
 *  Created on: 5 Jan 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"

extern QueueHandle_t buzzing_task_msg;
uint16_t g_motor_fault;

void set_motor_config(motor_data_t* motor){
	//general config:
		switch(motor->motor_type){
		case TYPE_M3508_ANGLE:
		case TYPE_M3508_STEPS:
		case TYPE_M3508:
			motor->angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
			motor->angle_pid.physical_max = M3508_MAX_RPM;
			motor->rpm_pid.physical_max = M3508_MAX_OUTPUT;
			break;
		case TYPE_M3508_NGEARBOX:
			motor->angle_data.gearbox_ratio = 1;
			motor->angle_pid.physical_max = M3508_MAX_RPM;
			motor->rpm_pid.physical_max = M3508_MAX_OUTPUT;

		break;
		case TYPE_GM6020:
		case TYPE_GM6020_720:
			motor->angle_data.gearbox_ratio = 0;
			motor->angle_pid.physical_max = GM6020_MAX_RPM;
			motor->rpm_pid.physical_max = GM6020_MAX_OUTPUT;
			break;
		case TYPE_M2006:
		case TYPE_M2006_STEPS:
		case TYPE_M2006_ANGLE:
			motor->angle_data.gearbox_ratio = M2006_GEARBOX_RATIO;
			motor->angle_pid.physical_max = M2006_MAX_RPM;
			motor->rpm_pid.physical_max = M2006_MAX_OUTPUT;
			break;
		default:
			break;
		}

}

extern motor_data_t can_motors[24];
void config_motors() {
	for (uint8_t i = 0; i <= 24; i++) {
		//reset all the values to 0
		can_motors[i].motor_type = 0;
		can_motors[i].rpm_pid.output = 0;
		can_motors[i].rpm_pid.integral = 0;
		can_motors[i].angle_pid.output = 0;
		can_motors[i].angle_pid.integral = 0;
		can_motors[i].angle_data.ticks = 0;
	}

	//initialise motor data
#ifdef FR_MOTOR_ID
	uint8_t motor_id = FR_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508_STEPS;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.center_ang = 0;
	can_motors[motor_id].angle_data.max_ang = 0;
	can_motors[motor_id].angle_data.min_ang = 0;
	can_motors[motor_id].rpm_pid.kp = CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki = CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd = CHASSIS_KD;
	can_motors[motor_id].angle_data.wheel_circ = WHEEL_CIRC;
	can_motors[motor_id].rpm_pid.int_max = CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = CHASSIS_MAX_CURRENT;
#endif

#ifdef FL_MOTOR_ID
	motor_id = FL_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.center_ang = 0;
	can_motors[motor_id].angle_data.max_ang = 0;
	can_motors[motor_id].angle_data.min_ang = 0;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_data.wheel_circ = WHEEL_CIRC;
	can_motors[motor_id].rpm_pid.kp = CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki = CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd = CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max = CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = CHASSIS_MAX_CURRENT;
#endif

#ifdef BL_MOTOR_ID
	motor_id = BL_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.center_ang = 0;
	can_motors[motor_id].angle_data.max_ang = 0;
	can_motors[motor_id].angle_data.min_ang = 0;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_data.wheel_circ = WHEEL_CIRC;
	can_motors[motor_id].angle_pid.kp = 0;
	can_motors[motor_id].angle_pid.ki = 0;
	can_motors[motor_id].angle_pid.kd = 0;
	can_motors[motor_id].angle_pid.int_max = 0;
	can_motors[motor_id].angle_pid.max_out = 0;
	can_motors[motor_id].rpm_pid.kp = CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki = CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd = CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max = CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = CHASSIS_MAX_CURRENT;
#endif

#ifdef BR_MOTOR_ID

	motor_id = BR_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.center_ang = 0;
	can_motors[motor_id].angle_data.max_ang = 0;
	can_motors[motor_id].angle_data.min_ang = 0;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_data.wheel_circ = WHEEL_CIRC;
	can_motors[motor_id].angle_pid.kp = 0;
	can_motors[motor_id].angle_pid.ki = 0;
	can_motors[motor_id].angle_pid.kd = 0;
	can_motors[motor_id].angle_pid.int_max = 0;
	can_motors[motor_id].angle_pid.max_out = 0;
	can_motors[motor_id].rpm_pid.kp = CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki = CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd = CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max = CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = CHASSIS_MAX_CURRENT;
#endif

#ifdef LFRICTION_MOTOR_ID
	motor_id = LFRICTION_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508_NGEARBOX;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_pid.physical_max = M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp = FRICTION_KP;
	can_motors[motor_id].rpm_pid.ki = FRICTION_KI;
	can_motors[motor_id].rpm_pid.kd = FRICTION_KD;
	can_motors[motor_id].rpm_pid.int_max = FRICTION_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out = FRICTION_MAX_CURRENT;
#endif

#ifdef RFRICTION_MOTOR_ID
	motor_id = RFRICTION_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_M3508_NGEARBOX;
	can_motors[motor_id].angle_pid.physical_max = M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp = FRICTION_KP;
	can_motors[motor_id].rpm_pid.ki = FRICTION_KI;
	can_motors[motor_id].rpm_pid.kd = FRICTION_KD;
	can_motors[motor_id].rpm_pid.int_max = FRICTION_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out = FRICTION_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.physical_max = M3508_MAX_OUTPUT;
#endif

#ifdef FEEDER_MOTOR_ID
	motor_id = FEEDER_MOTOR_ID - 1;
#ifdef ANGLE_FEEDER
	can_motors[motor_id].motor_type = TYPE_M3508_ANGLE;
#endif
#ifndef ANGLE_FEEDER
	can_motors[motor_id].motor_type = TYPE_M2006;
#endif
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.max_ang = PI * 5000; //so it can rotate 5000 times oops
	can_motors[motor_id].angle_data.min_ang = -PI * 5000;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_data.wheel_circ = 0;
	can_motors[motor_id].angle_pid.kp = FEEDER_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki = FEEDER_ANGLE_KI;
	can_motors[motor_id].angle_pid.kd = FEEDER_ANGLE_KD;
	can_motors[motor_id].angle_pid.int_max = FEEDER_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out = FEEDER_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp = FEEDER_KP;
	can_motors[motor_id].rpm_pid.ki = FEEDER_KI;
	can_motors[motor_id].rpm_pid.kd = FEEDER_KD;
	can_motors[motor_id].rpm_pid.int_max = FEEDER_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out = FEEDER_MAX_CURRENT;
#endif

#ifdef PITCH_MOTOR_ID
	motor_id = PITCH_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_GM6020;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.gearbox_ratio = 0;
	can_motors[motor_id].angle_data.center_ang = PITCH_CENTER;
	can_motors[motor_id].angle_data.max_ang = PITCH_MAX_ANG;
	can_motors[motor_id].angle_data.min_ang = PITCH_MIN_ANG;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_data.wheel_circ = 0;
	can_motors[motor_id].angle_pid.kp = PITCH_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki = PITCH_ANGLE_KI;
	can_motors[motor_id].angle_pid.kd = PITCH_ANGLE_KD;
	can_motors[motor_id].angle_pid.int_max = PITCH_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out = PITCH_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp = PITCHRPM_KP;
	can_motors[motor_id].rpm_pid.ki = PITCHRPM_KI;
	can_motors[motor_id].rpm_pid.kd = PITCHRPM_KD;
	can_motors[motor_id].rpm_pid.int_max = PITCHRPM_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = PITCH_MAX_CURRENT;
#endif

#ifdef YAW_MOTOR_ID
	motor_id = YAW_MOTOR_ID - 1;
	can_motors[motor_id].motor_type = TYPE_GM6020;
	set_motor_config(&can_motors[motor_id]);
	can_motors[motor_id].angle_data.gearbox_ratio = 0;	//1;
	can_motors[motor_id].angle_data.center_ang = YAW_CENTER;
	can_motors[motor_id].angle_data.max_ang = YAW_MAX_ANG;
	can_motors[motor_id].angle_data.min_ang = YAW_MIN_ANG;
	can_motors[motor_id].angle_data.phy_max_ang = 2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI; //angle before it overflows
	can_motors[motor_id].angle_data.wheel_circ = 0;
	can_motors[motor_id].angle_pid.kp = YAW_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki = YAW_ANGLE_KI;
	can_motors[motor_id].angle_pid.kd = YAW_ANGLE_KD;
	can_motors[motor_id].angle_pid.int_max = YAW_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out = YAW_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp = YAWRPM_KP;
	can_motors[motor_id].rpm_pid.ki = YAWRPM_KI;
	can_motors[motor_id].rpm_pid.kd = YAWRPM_KD;
	can_motors[motor_id].rpm_pid.int_max = YAWRPM_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out = YAW_MAX_CURRENT;
#endif
}

void bz_buzzer(uint8_t high, uint8_t low) {
	uint8_t temp_msg = bz_debug_high;
	for (uint8_t i = 0; i < high; i++) {
		xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
	}
	temp_msg = bz_debug_low;
	for (int8_t i = 0; i < low; i++) {
		xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
	}
	temp_msg = bz_debug_rest;
	xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
}

uint16_t check_motors() {
	uint16_t error = 0;
	uint32_t curr_time = get_microseconds();
	if (curr_time - can_motors[FR_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (0);
	}
	if (curr_time - can_motors[FL_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (1);
	}
	if (curr_time - can_motors[BL_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (2);
	}
	if (curr_time - can_motors[BR_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (3);
	}

	if (curr_time - can_motors[LFRICTION_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (4);

	}
	if (curr_time - can_motors[RFRICTION_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << (5);

	}
	if (curr_time - can_motors[FEEDER_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 6;

	}

	if (curr_time - can_motors[PITCH_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 7;

	}
	if (curr_time - can_motors[YAW_MOTOR_ID - 1].last_time[0] > MOTOR_TIMEOUT_MAX) {
		error |= 1 << 8;

	}
	return error;

}

void motor_calib_task(void *argument) {
	can_start(&hcan1, 0x00000000, 0x00000000);
	can_start(&hcan2, 0x00000000, 0x00000000);
	config_motors();
	//insert can tester?
	uint16_t error = 0b111111111;
	vTaskDelay(50);
	uint8_t startup = 0;
	if (MOTOR_ONLINE_CHECK == 1) {
		while (error != 0) {
			error = check_motors();
			for (uint8_t i = 0; i < 4; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(1, i + 1);
				}
			}
			for (uint8_t i = 4; i < 7; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(2, (i - 3));
				}
			}
			for (uint8_t i = 7; i < 9; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(3, (i - 6));
				}
			}
			vTaskDelay(500);
		}
	}

	uint8_t temp_msg;
	if (error == 0){
		temp_msg = ok;
	} else {
		temp_msg = not_ok;
	}
	xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);

	while (1) {
		error = 0;

		error = check_motors();
		g_motor_fault = error;

		if (MOTOR_ONLINE_CHECK == 1) {
			for (uint8_t i = 0; i < 4; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(1, i + 1);
				}
			}
			for (uint8_t i = 4; i < 7; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(2, (i - 3));
				}
			}
			for (uint8_t i = 7; i < 9; i++) {
				if (error & (1 << (i))) {
					bz_buzzer(3, (i - 6));
				}
			}
			vTaskDelay(5000);
			continue;
		}
		else if (MOTOR_ONLINE_CHECK == 0) {
			if(error != 0) {
			bz_buzzer(0, 2);
			vTaskDelay(5000);
			continue;
			}
		}
		vTaskDelay(10000);
	}

	//future calibration code, if any
	// task takes highest priority over....everything so make sure to kill all motors first!
	//implement mutexes so this task doesn't check while the motor tasks do their thing

	//write in task here to calibrate then stop lol
}

