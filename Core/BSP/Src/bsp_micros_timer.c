/*
 * bsp_micros_timer.c
 *
 *  Created on: Jan 17, 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"

uint32_t gv_curr_micros = 0;
uint32_t gv_overflow_times = 0;

void micros_tick(){
	if (gv_curr_micros == 4294967295) //2^32 - 1
		gv_overflow_times ++;
	gv_curr_micros ++;
}

void start_micros_timer()
{
	configASSERT(TIMER_FREQ <= 1000000);
	__HAL_TIM_SET_AUTORELOAD(&htim3, (999999/TIMER_FREQ) + 1);
	htim3.Instance->DIER |= TIM_DIER_UIE; //enable update interrupt
	HAL_TIM_Base_Start(&htim3);
}



uint32_t get_microseconds()
{
	return gv_curr_micros;
}
