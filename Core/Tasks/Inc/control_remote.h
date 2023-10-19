/*
 * control_remote.h
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#ifndef TASKS_INC_CONTROL_REMOTE_H_
#define TASKS_INC_CONTROL_REMOTE_H_

void remote_control_input();
void remote_gear_shifter(speed_shift_t* gear_speed);
void remote_chassis_input();
void remote_gimbal_input();
void remote_launcher_control_input();

#endif /* TASKS_INC_CONTROL_REMOTE_H_ */
