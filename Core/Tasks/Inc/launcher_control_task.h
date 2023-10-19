/*
 * launcher_control_task.h
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_LAUNCHER_CONTROL_TASK_H_
#define TASKS_INC_LAUNCHER_CONTROL_TASK_H_


void launcher_control_task(void *argument);
void launcher_control(motor_data_t *left_friction_motor, motor_data_t *right_friction_motor, motor_data_t *feeder);
void launcher_pid_init();


#endif /* TASKS_INC_LAUNCHER_CONTROL_TASK_H_ */
