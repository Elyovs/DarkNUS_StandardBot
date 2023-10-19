/*
 * startup_task.h
 *
 *  Created on: 23 May 2021
 *      Author: wx
 */

#ifndef TASKS_INC_STARTUP_TASK_H_
#define TASKS_INC_STARTUP_TASK_H_


void startup_task();
void err_buzzer(uint8_t low, uint8_t high);
void ok_buzzer(uint8_t high, uint8_t low);

#endif /* TASKS_INC_STARTUP_TASK_H_ */
