/*
 * bsp_micros_timer.h
 *
 *  Created on: Jan 17, 2022
 *      Author: wx
 */

#ifndef BSP_INC_BSP_MICROS_TIMER_H_
#define BSP_INC_BSP_MICROS_TIMER_H_


void micros_tick();
void start_micros_timer();
uint32_t get_microseconds();

#endif /* BSP_INC_BSP_MICROS_TIMER_H_ */
