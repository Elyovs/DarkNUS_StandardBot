/*
 * motor_control.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "motor_control.h"
#include "robot_config.h"

//todo: clean this place
void yaw_pid(double setpoint, double curr_pt, pid_data_t *pid) {
	pid->last_time[1] = pid->last_time[0];
	pid->last_time[0] = get_microseconds();
	if (pid->last_time[1]>= pid->last_time[0]) {
		return;
	}
	uint32_t time_mult = 1;//TIMER_FREQ / (float) (pid->last_time[0] - pid->last_time[1]);
	double Pout = 0;
	double Iout = 0;
	double Dout = 0;

	pid->error[1] = pid->error[0];
	pid->error[0] = setpoint - curr_pt;
	Pout = pid->error[0] * pid->kp * time_mult;
	Dout = (float)(pid->error[0] - pid->error[1]) * pid->kd * time_mult;
	pid->integral += pid->error[0] * pid->ki * time_mult;
	float_max(&pid->integral, pid->int_max);
	Iout = pid->integral;
	pid->output = Pout + Iout + Dout;
	float_max(&pid->output, pid->max_out);
}


/* alternative yaw function that uses the imu data to calculate RPM instead of raw RPM values
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data
 * for target motor
 *
 */

void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data) {
	double ang_diff = (setpoint - curr_pt);
	if (ang_diff > PI) {
		ang_diff -= 2 * PI;
	} else if (ang_diff < -PI) {
		ang_diff += 2 * PI;
	}

	if (*prev_imu_data == imu_data) {
		return;}
	motor->angle_pid.last_time[1] = motor->angle_pid.last_time[0];
	motor->angle_pid.last_time[0] = get_microseconds();

	uint32_t time_mult = TIMER_FREQ * 60 /
			(float) (motor->angle_pid.last_time[0] - motor->angle_pid.last_time[1]);
	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = ang_diff;
	float rpm_pOut = motor->angle_pid.kp * ang_diff;
	float rpm_dOut = motor->angle_pid.kd * (motor->angle_pid.error[0] - motor->angle_pid.error[1]);

	float imu_ang_diff = imu_data - *prev_imu_data;
	imu_ang_diff = (imu_ang_diff > PI) ? imu_ang_diff - (2 * PI) :
			((imu_ang_diff < -PI) ? imu_ang_diff + (2*PI) : imu_ang_diff);
	float imu_rpm = ((imu_data - *prev_imu_data)  * time_mult)/(2 * PI);
	*prev_imu_data = imu_data;

	motor->angle_pid.integral += motor->angle_pid.error[0];
	float_max(&motor->angle_pid.integral, motor->angle_pid.int_max);
	float rpm_iOut = motor->angle_pid.ki * motor->angle_pid.integral;

	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_max(&motor->angle_pid.output, motor->angle_pid.max_out);
	yaw_pid(motor->angle_pid.output, imu_rpm, &motor->rpm_pid);
}

/* Function for angle PID (i.e. aiming for a target angle rather than RPM)
 * Function calculates target RPM, then calls the speed PID
 * function to set the motor's rpm until it reaches the target angle
 *
 * @param setpoint target value
 * @param curr_pt current angle
 * @param *motor pointer to the struct that contain the data
 * for target motor
 *
 */
void angle_pid(double setpoint, double curr_pt, motor_data_t *motor) {
	double ang_diff = (setpoint - curr_pt);
	if (ang_diff > PI) {
		ang_diff -= 2 * PI;
	} else if (ang_diff < -PI) {
		ang_diff += 2 * PI;
	}
	motor->angle_pid.error[1] = motor->angle_pid.error[0];
	motor->angle_pid.error[0] = ang_diff;
	float rpm_pOut = motor->angle_pid.kp * ang_diff;
	float rpm_dOut = motor->angle_pid.kd * (motor->angle_pid.error[0] - motor->angle_pid.error[1]);

	motor->angle_pid.integral += motor->angle_pid.error[0];
	float_max(&motor->angle_pid.integral, motor->angle_pid.int_max);
	float rpm_iOut = motor->angle_pid.ki * motor->angle_pid.integral;
	motor->angle_pid.output = rpm_pOut + rpm_dOut + rpm_iOut;
	float_max(&motor->angle_pid.output, motor->angle_pid.max_out);
	speed_pid(motor->angle_pid.output, motor->raw_data.rpm, &motor->rpm_pid);
}

/*
 * Function for speed PID
 * For motors that might see constant torque, i.e. chassis motors
 * make sure an integral value is initialised (VERY SMALL, like 0.0001 or smaller)
 * as their systems usually have a steady state error
 *
 *
 * @param setpoint target RPM
 * @param motor's current RPM
 * @param *pid pointer to the rpm_pid struct within the motor's data struct
 */
void speed_pid(double setpoint, double curr_pt, pid_data_t *pid) {
	pid->last_time[1] = pid->last_time[0];
	pid->last_time[0] = get_microseconds();
	double Pout = 0;
	double Iout = 0;
	double Dout = 0;

	pid->error[1] = pid->error[0];
	pid->error[0] = setpoint - curr_pt;
	Pout = pid->error[0] * pid->kp;
	Dout = (float)(pid->error[0] - pid->error[1]) * pid->kd;
	pid->integral += pid->error[0] * pid->ki;
	float_max(&pid->integral, pid->int_max);
	Iout = pid->integral;
	pid->output = Pout + Iout + Dout;
	float_max(&pid->output, pid->max_out);
}

/*
 * Function to send commands
 * To use, input the motor ids of the motors desired
 * it will then send the number in the rpm_pid.output of the motors
 * to the motors
 * Motors with the same CAN header will also be sent the last output value put in their
 * pid struct
 *
 * @param motor_all[] pointer to the array that contains the data for *all* the motors
 * @param id_one	id number of first motor to send can message to
 * @param id_two	id number of second motor to send can message to
 * @param id_three	id number of third motor to send can message to
 * @param id_four	id number of fourth motor to send can message to
 */
void motor_send_can(motor_data_t motor_all[],
		uint8_t id_one,
		uint8_t id_two,
		uint8_t id_three,
		uint8_t id_four) {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box;
	uint32_t temp_checker = 0;
	int16_t temp_converter;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
	if (id_one < 25 && id_one > 0) {
		temp_checker = temp_checker | 1 << (id_one - 1);
	}
	if (id_two < 25 && id_two > 0) {
		temp_checker = temp_checker | 1 << (id_two - 1);
	}
	if (id_three < 25 && id_three > 0) {
		temp_checker = temp_checker | 1 << (id_three - 1);
	}
	if (id_four < 25 && id_four > 0) {
		temp_checker = temp_checker | 1 << (id_four - 1);
	}

	if (temp_checker & 0x00000F) {
		CAN_tx_message.StdId = 0x200;
		temp_converter = motor_all[0x0].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x1].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x2].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x3].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x0000F0) {
		CAN_tx_message.StdId = 0x1FF;
		temp_converter = motor_all[0x4].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x5].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x6].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x7].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x000F00) {
		CAN_tx_message.StdId = 0x2FF;
		temp_converter = motor_all[0x8].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x9].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0xA].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0xB].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
#ifndef CHASSIS_MCU
	if (temp_checker & 0x00F000) {
		CAN_tx_message.StdId = 0x200;
		temp_converter = motor_all[0x0 + 12].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x1 + 12].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x2 + 12].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x3 + 12].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0x0F0000) {
		CAN_tx_message.StdId = 0x1FF;
		temp_converter = motor_all[0x4 + 12].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x5 + 12].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0x6 + 12].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0x7 + 12].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
	if (temp_checker & 0xF00000) {
		CAN_tx_message.StdId = 0x2FF;
		temp_converter = motor_all[0x8 + 12].rpm_pid.output;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = motor_all[0x9 + 12].rpm_pid.output;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = motor_all[0xA + 12].rpm_pid.output;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = motor_all[0xB + 12].rpm_pid.output;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
#endif
}

void kill_can() {

	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8] = { 0, };
	uint32_t send_mail_box;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
//
//	CAN_tx_message.StdId = 0x200;
//	HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
//	CAN_tx_message.StdId = 0x1FF;
//	HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
//	CAN_tx_message.StdId = 0x2FF;
//	HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
//	CAN_tx_message.StdId = 0x200;
//#ifndef CHASSIS_MCU
//	HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
//	CAN_tx_message.StdId = 0x1FF;
//	HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
//	CAN_tx_message.StdId = 0x2FF;
//	HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
//
//#endif
}

/**
 * Limits the input float variable
 * @params motor_in: the pointer to the variable to be limited
 * @params motor_max: the positive maximum value for the variable
 */

void float_max(float *motor_in, float motor_max) {
	if (*motor_in > motor_max) {
		*motor_in = motor_max;
	} else if (*motor_in < -motor_max) {
		*motor_in = -motor_max;
	}
}

/**
 * Resets PID values for the motors using the motor_data_t struct
 * @params motor_data: pointer to the motor data struct
 *
 * todo: add in one to reset gimbal struct
 */
void reset_pid(motor_data_t *motor_data) {

}

