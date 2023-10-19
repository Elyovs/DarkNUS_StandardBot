/*
 * movement_control_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_MOVEMENT_CONTROL_TASK_H_
#define TASKS_INC_MOVEMENT_CONTROL_TASK_H_


void movement_control_task(void *argument);
void chassis_motion_control();
void chassis_pid_init();

#endif /* TASKS_INC_MOVEMENT_CONTROL_TASK_H_ */
