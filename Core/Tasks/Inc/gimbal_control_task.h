/*
 * gimbal_control_task.h
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#ifndef TASKS_INC_GIMBAL_CONTROL_TASK_H_
#define TASKS_INC_GIMBAL_CONTROL_TASK_H_

void gimbal_control_task(void *argument);
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);
void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);
void gimbal_pid_init();


#endif /* TASKS_INC_GIMBAL_CONTROL_TASK_H_ */
