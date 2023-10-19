/*
 * control_keyboard.h
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#ifndef TASKS_INC_CONTROL_KEYBOARD_H_
#define TASKS_INC_CONTROL_KEYBOARD_H_

void keyboard_control_input();
void keyboard_gear_shifter(speed_shift_t *gear_speed);
void keyboard_chassis_input();
void mouse_gimbal_input();
void mouse_launcher_control_input();

#endif /* TASKS_INC_CONTROL_KEYBOARD_H_ */
