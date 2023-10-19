/*
 * actuator_feedback_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_CAN_MSG_PROCESSOR_H_
#define TASKS_INC_CAN_MSG_PROCESSOR_H_



void convert_raw_can_data(motor_data_t * can_motor_data, uint16_t motor_id, uint8_t *rx_buffer);
void motor_send_can(motor_data_t motor_all[],uint8_t id_one,
		uint8_t id_two, uint8_t id_three, uint8_t id_four);
void actuator_feedback_task(void *argument);
void angle_offset(raw_data_t *motor_data, angle_data_t *angle_data);

void motor_calc_odometry(raw_data_t *motor_data, angle_data_t *angle_data, uint32_t feedback_times[]);

#endif /* TASKS_INC_CAN_MSG_PROCESSOR_H_ */
