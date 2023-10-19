/*
 * actuator_feedback_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "can_msg_processor.h"

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;
#define ANGLE_LPF 0
#define SPEED_LPF 0
#ifndef CHASSIS_MCU
motor_data_t can_motors[24];
#else
motor_data_t can_motors[12];
#endif


/**
 * CAN ISR function, triggered upon RX_FIFO0_MSG_PENDING
 * converts the raw can data to the motor_data struct form as well
 */
void can_ISR(CAN_HandleTypeDef *hcan) {

	CAN_RxHeaderTypeDef rx_msg_header;
	uint8_t rx_buffer[CAN_BUFFER_SIZE];
	//check which CAN bus received it
	//required because the 2 canbuses use seperate FIFOs for receive
	if (hcan->Instance == CAN1) {
		HAL_CAN_DeactivateNotification(hcan,
				CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
		can1_get_msg(&hcan1, &rx_msg_header, rx_buffer);
		convert_raw_can_data(can_motors, rx_msg_header.StdId, rx_buffer);
		HAL_CAN_ActivateNotification(hcan,
				CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
	}
#ifndef CHASSIS_MCU
	else if (hcan->Instance == CAN2) {
		HAL_CAN_DeactivateNotification(hcan,
				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
		can2_get_msg(&hcan2, &rx_msg_header, rx_buffer);
		//StdId +12 to seperate the motors on CAN1 and CAN2
		convert_raw_can_data(can_motors, rx_msg_header.StdId + 12, rx_buffer);
		HAL_CAN_ActivateNotification(hcan,
				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
	}
#else
	else if (hcan->Instance == CAN2)
	{
		can_get_msg(&hcan2, &rx_msg_header, rx_buffer);
		process_chassis_can_msg(rx_msg_header.StdId, rx_buffer);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL	| CAN_IT_RX_FIFO0_OVERRUN);
	}
#endif
}

/*
 * Converts raw CAN data over to the motor_data_t struct
 * 7 bytes of CAN data is sent from the motors:
 * High byte for motor angle data
 * Low byte for motor angle data
 * High byte for RPM
 * Low byte for RPM
 * High byte for Torque
 * Low byte for Torque
 * 1 byte for temperature
 *
 * This function combines the respective high and low bytes into 1 single 16bit integer, then stores them
 * in the struct for the motor.
 *
 * For GM6020 motors, it recenters the motor angle data and converts it to radians.
 */

void convert_raw_can_data(motor_data_t *can_motor_data, uint16_t motor_id, uint8_t *rx_buffer) {
	uint16_t idnum = motor_id - 0x201;

	//if idnum > 24, it's not a DJI motor. Add in a seperate processing function if other CAN devices are added
	if (idnum > 24)
	{
		return;
	}
	motor_data_t* curr_motor = &can_motor_data[idnum];
	//motor must be initialised in motor_config.c first
	if (curr_motor->motor_type > 0) {
		//convert the raw data back into the respective values
		curr_motor->id 				= motor_id;
		curr_motor->raw_data.angle[0] = (rx_buffer[0] << 8) | rx_buffer[1];
		int16_t temp_rpm						=(rx_buffer[2] << 8) | rx_buffer[3];
		curr_motor->raw_data.rpm 		= curr_motor->raw_data.rpm * SPEED_LPF + temp_rpm * (1-SPEED_LPF);
		curr_motor->raw_data.torque 	= (rx_buffer[4] << 8) | rx_buffer[5];
		curr_motor->raw_data.temp 	= (rx_buffer[6]);
		curr_motor->last_time[1] 		= curr_motor->last_time[0];
		curr_motor->last_time[0]	 	= get_microseconds();

		float rds_passed = (float)(curr_motor->raw_data.angle[0] - curr_motor->raw_data.angle[1]) /8192;
		curr_motor->angle_data.hires_rpm = (rds_passed * TIMER_FREQ
				/ curr_motor->last_time[0] - curr_motor->last_time[1]) * 60;
		//process the angle data differently depending on the motor type to get radians in the
		//adj_angle value
		switch (curr_motor->motor_type) {
		case TYPE_GM6020:
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		case TYPE_M2006:
		case TYPE_M3508:
			break;
		case TYPE_M2006_STEPS:
		case TYPE_M3508_STEPS:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			break;
		case TYPE_M2006_ANGLE:
		case TYPE_M3508_ANGLE:
		case TYPE_GM6020_720:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		default:
			break;

		}

		//initialise task switching variables
		BaseType_t xHigherPriorityTaskWoken, xResult;
		xHigherPriorityTaskWoken = pdFALSE;

		//set event group bits so that the tasks and PIDs only trigger upon updated data
		//also checks if the respective tasks are set to ready
		switch (idnum + 1) {
		#ifndef CHASSIS_MCU
				case FR_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b1000,
							&xHigherPriorityTaskWoken);
					break;
				case FL_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0100,
							&xHigherPriorityTaskWoken);
					break;
				case BL_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0010,
							&xHigherPriorityTaskWoken);
					break;
				case BR_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0001,
							&xHigherPriorityTaskWoken);
					break;
		#endif
				case LFRICTION_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b010,
							&xHigherPriorityTaskWoken);
					break;
				case RFRICTION_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b001,
							&xHigherPriorityTaskWoken);
					break;
				case FEEDER_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b100,
							&xHigherPriorityTaskWoken);
					break;
				case PITCH_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
							&xHigherPriorityTaskWoken);
					break;
				case YAW_MOTOR_ID:
					xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b10,
							&xHigherPriorityTaskWoken);
					break;
				default:
					idnum = idnum;
					//error handler
					break;
				}

				//switches tasks if a higher priority task is ready.
				//required because the function is in an ISR
				if (xResult != pdFAIL) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //forces current task to yield if higher priority task is called
				}
			} else {
				//this is a useless statement so that it is possible to set a breakpoint here lol
				uint8_t stopper = 0;
				//error handler
		}
}

void process_chassis_can_msg(uint16_t msg_id, uint8_t rx_buffer[]) {
	//for future use
}

/**
 * Centers the raw motor angle to between -Pi to +Pi
 */
void angle_offset(raw_data_t *motor_data, angle_data_t *angle_data) {
	int32_t temp_ang = 0;

	//if there's a gearbox, use the ticks after the gearbox.
	//make sure center angle is properly set with respect to the zero-ing angle
	//YOUR ROBOT MUST HAVE A WAY TO ZERO THIS ANGLE AND IMPLEMENT A ZEROING FUNCTION AT STARTUP
	//IF NOT IT WON'T WORK 							-wx
	if (angle_data->gearbox_ratio > 0) {
		temp_ang = angle_data->ticks - angle_data->center_ang;
		if (temp_ang > (4096 * angle_data->gearbox_ratio)) {
			temp_ang -= (8192 * angle_data->gearbox_ratio);
		} else if (temp_ang < (-4096 * angle_data->gearbox_ratio)) {
			temp_ang += 8192 * angle_data->gearbox_ratio;
		}
		angle_data->ticks = temp_ang;
		angle_data->adj_ang = (float) temp_ang * 2 *PI / (8192 * angle_data->gearbox_ratio);
	} else {
		//for motors without gearbox, the angle data can be directly converted into radians
		temp_ang = (int32_t) (motor_data->angle[0]) - angle_data->center_ang;
		//to account for center angles not at 4096 (mechies make our lives easy pls)
		//if the angle isn't within -4096 to 4096 add or subtract 8192 from the value
		if (temp_ang > 4096) {
			temp_ang -= 8192;
		} else if (temp_ang < -4096) {
			temp_ang += 8192;
		}
		angle_data->adj_ang = (angle_data->adj_ang * ANGLE_LPF) + (float) (temp_ang * PI / 4096) * (1 - ANGLE_LPF); // convert to radians
	}
}

void motor_calc_odometry(raw_data_t *motor_data,
		angle_data_t *angle_data,
		uint32_t feedback_times[]) {
	int8_t int_round_passed = 0;
	//check to make sure it's not the same data point
	if (feedback_times[0] - feedback_times[1] >= 1) {
		float rounds_passed = (((float)(feedback_times[0] - feedback_times[1]) * motor_data->rpm)/(60 * TIMER_FREQ));
		if (fabs(rounds_passed) >= 1) {
			int_round_passed = rounds_passed;
		} else {
			int_round_passed = 0;
		}
	}
	int16_t abs_angle_diff;
	abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	//generally the motor won't exceed half a turn between each feedback
	if (abs_angle_diff > 4096) {
		abs_angle_diff -= 8192;
	} else if (abs_angle_diff < -4096) {
		abs_angle_diff += 8192;
	}

	// check if motor has turned more than half a round, and adds it to the number of ticks
	//commented out because RPM measurement on the motor usually lags behind position measurement, and it's not really needed
	/*
	 else if (angle_data->dir == 1) {
	 if (motor_data->angle[0] > motor_data->angle[1]) {
	 abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	 } else {
	 abs_angle_diff = motor_data->angle[1] + (8192-motor_data->angle[1]);
	 }
	 } else {
	 if (motor_data->angle[0] < motor_data->angle[1]) {
	 abs_angle_diff = motor_data->angle[1] - motor_data->angle[0];
	 } else {
	 abs_angle_diff = (8192-motor_data->angle[0]) - motor_data->angle[1];
	 }
	 }
	 */
	uint16_t gear_ticks = 8192 * angle_data->gearbox_ratio;
	angle_data->ticks += (int_round_passed * 8192) + abs_angle_diff;
	angle_data->dist = angle_data->ticks * angle_data->wheel_circ / gear_ticks;
	angle_data->adj_ang = (float) (angle_data->ticks % gear_ticks ) * 2*PI/gear_ticks;
	angle_data->adj_ang = (angle_data->adj_ang > PI) ? (angle_data->adj_ang - 2*PI) : (angle_data->adj_ang < -PI) ? angle_data->adj_ang+2*PI : angle_data->adj_ang;
	motor_data->angle[1] = motor_data->angle[0];
}
