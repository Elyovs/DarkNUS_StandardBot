/*
 * gimbal_control_task.c
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "gimbal_control_task.h"

extern uint8_t aimbot_mode;

extern EventGroupHandle_t gimbal_event_group;
extern float g_chassis_yaw;
extern motor_data_t can_motors[24];
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern QueueHandle_t telem_motor_queue;

/**
 *
 * FreeRTOS task for gimbal controls
 * Has HIGH2 priority
 *
 */
void gimbal_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
		xEventGroupWaitBits(gimbal_event_group, 0b11, pdTRUE, pdTRUE,
		portMAX_DELAY);
		start_time = xTaskGetTickCount();
		if (gimbal_ctrl_data.enabled) {
			if (gimbal_ctrl_data.imu_mode) {
				gimbal_control(can_motors + PITCH_MOTOR_ID - 1,
						can_motors + YAW_MOTOR_ID - 1);
			} else {
				gimbal_angle_control(can_motors + PITCH_MOTOR_ID - 1,
						can_motors + YAW_MOTOR_ID - 1);
			}
		} else {
			can_motors[PITCH_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[YAW_MOTOR_ID - 1].rpm_pid.output = 0;
			motor_send_can(can_motors, PITCH_MOTOR_ID, YAW_MOTOR_ID, 0, 0);
		}
		status_led(2, off_led);
		xEventGroupClearBits(gimbal_event_group, 0b11);
		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
}

/**
 * This function controls the gimbals based on IMU reading
 * @param 	pitch_motor		Pointer to pitch motor struct
 * 			yaw_motor		Pointer to yaw motor struct
 * @note both pitch and yaw are currently on CAN2 with ID5 and 6.
 * Need to check if having ID4 (i.e. 0x208) + having the launcher motors (ID 1-3, 0x201 to 0x203)
 * still provides a fast enough response
 */
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {
	static float prev_pit;
	static float prev_yaw;
	//todo: add in roll compensation
	uint8_t pit_lim = 0;
	uint8_t yaw_lim = 0;
	float rel_pitch_angle = pitch_motor->angle_data.adj_ang
			+ gimbal_ctrl_data.pitch - imu_heading.pit;
	if (rel_pitch_angle > pitch_motor->angle_data.max_ang) {
		rel_pitch_angle = pitch_motor->angle_data.max_ang;
		pit_lim = 1;
	}
	if (rel_pitch_angle < pitch_motor->angle_data.min_ang) {
		rel_pitch_angle = pitch_motor->angle_data.min_ang;
		pit_lim = 1;
	}
	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle + imu_heading.pit
				- pitch_motor->angle_data.adj_ang;
	}

	yangle_pid(gimbal_ctrl_data.pitch,imu_heading.pit, pitch_motor,
			imu_heading.pit, &prev_pit);


	float rel_yaw_angle = yaw_motor->angle_data.adj_ang + gimbal_ctrl_data.yaw
			- imu_heading.yaw;
	//if yaw has overflowed (i.e. goes to the next round) move it back into pi to -pi range
	if (rel_yaw_angle > PI) {
		rel_yaw_angle -= 2 * PI;
	}
	if (rel_yaw_angle < -PI) {
		rel_yaw_angle += 2 * PI;
	}
	//check limits
	if (rel_yaw_angle > yaw_motor->angle_data.max_ang) {
		rel_yaw_angle = yaw_motor->angle_data.max_ang;
		yaw_lim = 1;
	}
	if (rel_yaw_angle < yaw_motor->angle_data.min_ang) {
		rel_yaw_angle = yaw_motor->angle_data.min_ang;
		yaw_lim = 1;
	}
	if (yaw_lim == 1) {
		gimbal_ctrl_data.yaw = rel_yaw_angle + imu_heading.yaw
				- yaw_motor->angle_data.adj_ang;

	}

//	yangle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor,
//			imu_heading.yaw, &prev_yaw);
	yangle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor,
			imu_heading.yaw, &prev_yaw);
//	angle_pid(gimbal_ctrl_data.yaw, imu_heading.yaw, yaw_motor);

	motor_send_can(can_motors, PITCH_MOTOR_ID, YAW_MOTOR_ID, 0, 0);


}

void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {

	if (gimbal_ctrl_data.pitch > pitch_motor->angle_data.max_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.pitch < pitch_motor->angle_data.min_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.min_ang;
	}

	if (gimbal_ctrl_data.yaw > yaw_motor->angle_data.max_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.yaw < yaw_motor->angle_data.min_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.min_ang;
	}
	angle_pid(gimbal_ctrl_data.pitch, pitch_motor->angle_data.adj_ang,
			pitch_motor);
	angle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor);
	motor_send_can(can_motors, PITCH_MOTOR_ID, YAW_MOTOR_ID, 0, 0);
}
